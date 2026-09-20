/**
 ******************************************************************************
 * @file    debug_cmd.c
 * @brief   USART3 调试命令行实现
 * @date    2026-08-03
 ******************************************************************************
 * @attention
 * 文本命令行格式: <命令> [参数1] [参数2] ...
 * 行缓冲 128 字节, 支持 Backspace (0x7F) 退格
 * 回车 (0x0D) 或换行 (0x0A) 执行
 ******************************************************************************
 */

#include "debug_cmd.h"
#include "app_threads.h"
#include "DrEmpower_can.h"
#include "servo.h"
#include "sys_status.h"
#include "usart.h"
#include "can.h"
#include "safety.h"
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 外部依赖 — CAN 互斥量 (app_threads.c 中定义) */
extern rt_mutex_t mutex_can_tx;
extern rt_mutex_t mutex_can_rx;

/* ======================================================================== */
/*   浮点格式化 (rt_kprintf 不支持 %f)                                       */
/* ======================================================================== */
#define FTOA_BUF_COUNT  8
static char ftoa_buf[FTOA_BUF_COUNT][24];
static int  ftoa_idx = 0;

/**
 * @brief float → string, rt_kprintf 兼容 (%.1f ~ %.4f 精度)
 * @param v  浮点值
 * @param dp 小数位数 (1~4)
 * @return 静态缓冲区字符串指针 (轮转, 最多同时用 4 个)
 */
static const char *ftoa(float v, int dp)
{
    if (dp < 0) dp = 0; if (dp > 6) dp = 6;
    char *buf = ftoa_buf[ftoa_idx];
    ftoa_idx = (ftoa_idx + 1) % FTOA_BUF_COUNT;

    int neg = (v < 0.0f);
    if (neg) v = -v;

    /* 四舍五入 */
    float round_val = 0.5f;
    for (int i = 0; i < dp; i++) round_val *= 0.1f;
    v += round_val;

    int int_part = (int)v;
    /* 保证 int_part 非负 */
    if (int_part < 0) int_part = 0;
    if (int_part > 999) int_part = 999;

    float frac_f = v - (float)int_part;
    if (frac_f < 0.0f) frac_f = 0.0f;
    int frac = (int)(frac_f * 1000000.0f + 0.5f);
    if (frac < 0) frac = 0;
    if (frac > 999999) frac = 999999;
    int div = 1;
    for (int i = 0; i < 6 - dp; i++) div *= 10;
    frac /= div;

    int pos = 0;
    if (neg) buf[pos++] = '-';
    else     buf[pos++] = ' ';
    buf[pos++] = '0' + (int_part / 100) % 10;
    buf[pos++] = '0' + (int_part / 10) % 10;
    buf[pos++] = '0' + (int_part % 10);
    if (dp > 0) {
        buf[pos++] = '.';
        int fw = 1;
        for (int i = 1; i < dp; i++) fw *= 10;
        for (int d = fw; d >= 1; d /= 10)
            buf[pos++] = '0' + ((frac / d) % 10);
    }
    buf[pos] = '\0';
    return buf;
}

/* ======================================================================== */
/*   行缓冲与解析                                                            */
/* ======================================================================== */
#define CMD_BUF_SIZE    128
#define CMD_MAX_ARGS    8

static char   cmd_buf[CMD_BUF_SIZE];
static int    cmd_pos = 0;

/* 解析后的参数 */
static int    argc = 0;
static char  *argv[CMD_MAX_ARGS];
static char   argv_buf[CMD_BUF_SIZE];  /* strtok 可写副本 */

/* Pi 原始字节追踪开关 */
volatile int g_raw_monitor = 0;

/* CAN 测试帧发送开关 */
static volatile int g_can_test = 0;
static uint8_t g_can_test_seq = 0;

/* ======================================================================== */
/*   命令实现                                                               */
/* ======================================================================== */

/* ---- 辅助宏 ---- */
#define NEED_ARGS(n)  if (argc < (n)) { rt_kprintf("  Usage: %s\n", _usage); return; }

static const char *_usage;  /* 当前命令用法 */

/* 安全保护: J2/J3/J4 正值会撞机，拦截 */
static int _positive_block(uint8_t id, float angle)
{
    if ((id == 2 || id == 3 || id == 4) && angle > 0.0f) {
        rt_kprintf("  BLOCKED: J%d positive angle (%.2f°) rejected — collision risk\n", id, angle);
        return 1;
    }
    return 0;
}

/* ---- h / help ---- */
static void cmd_help(void)
{
    rt_kprintf("\n");
    rt_kprintf("========== DaRan Robot Arm Debug Console ==========\n");
    rt_kprintf("  h, help                This help\n");
    rt_kprintf("  s, status              System status + all joints\n");
    rt_kprintf("  info <id>              Full joint info (angle/speed/torque/vol/cur/pid)\n");
    rt_kprintf("--- Joint Control ---\n");
    rt_kprintf("  j<id>                   Read joint angle (e.g. j1, j3)\n");
    rt_kprintf("  j<id> <angle> [speed]   Move joint to angle (e.g. j1 90, j2 -30 60)\n");
    rt_kprintf("  m  <j1>..<j6> [spd]    6-joint PTP (e.g. m 0 0 0 0 0 0 60)\n");
    rt_kprintf("  e, estop [id]           EStop (no arg=all, with arg=single)\n");
    rt_kprintf("  clear                   Clear ESTOP & error, resume running\n");
    rt_kprintf("  z, zero                 Home: all joints to 0\n");
    rt_kprintf("  ready                   Move all joints to ready position\n");
    rt_kprintf("  home                    Move to home (reverse: ready→home)\n");
    rt_kprintf("  home 1                  Move to home (forward: zero→home)\n");
    rt_kprintf("  g  <angle> [id]         Gripper (e.g. g 120, g 50 3)\n");
    rt_kprintf("--- Calibration ---\n");
    rt_kprintf("  cal  <id>               Start calibrate joint (force-assist)\n");
    rt_kprintf("  calz <id>               Confirm set zero + readback\n");
    rt_kprintf("  cal_end                  Exit calibration mode\n");
    rt_kprintf("--- Settings ---\n");
    rt_kprintf("  setid <old> <new>       Set joint CAN ID (single joint on bus!)\n");
    rt_kprintf("  setzero <id>            Set zero (current pos = 0°, permanent)\n");
    rt_kprintf("  pid <id> <p> <i> <d>   Set joint PID\n");
    rt_kprintf("  reboot <id>             Reboot joint\n");
    rt_kprintf("  save <id>               Save joint config to flash\n");
    rt_kprintf("  speed <id> <speed>      Set joint speed limit\n");
    rt_kprintf("  torque <id> <torque>    Set joint torque limit\n");
    rt_kprintf("  mode <id> <1|2>         Set joint mode (1=idle 2=closed-loop)\n");
    rt_kprintf("--- Monitoring ---\n");
    rt_kprintf("  raw on/off              Toggle Pi UART raw byte monitor\n");
    rt_kprintf("  canmon on/off           Toggle CAN bus raw frame monitor\n");
    rt_kprintf("  pi  on/off              Toggle Pi UART passthrough\n");
    rt_kprintf("  stream <rate_ms> [0]    Start/stop telemetry stream\n");
    rt_kprintf("--- CAN Test ---\n");
    rt_kprintf("  cantest 1/0             CAN loopback: ID=0x1801AA, data seq++/1s\n");
    rt_kprintf("=====================================================\n\n");
}

/* ---- s / status ---- */
static void cmd_status(void)
{
    rt_kprintf("\n[SYS] State=%d Err=%d ESTOP=%d Motion=%d Uptime=%lums\n",
               g_sys.state, g_sys.error_code,
               g_sys.estop_triggered, g_sys.motion_busy,
               g_sys.uptime_ms);

    rt_kprintf("[SYS] Zero Valid: ");
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        rt_kprintf("J%d=%c ", i+1, g_sys.zero_valid[i] ? 'Y' : 'N');
    rt_kprintf("\n");

    rt_kprintf("[SYS] Joint Status:\n");
    rt_kprintf("  %-6s %10s %10s %10s %8s %8s\n",
               "ID", "Angle(°)", "Speed(rpm)", "Torque(Nm)", "Vol(V)", "Cur(A)");
    rt_kprintf("  ------ ---------- ---------- ---------- -------- --------\n");
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        struct joint_data_msg *jd = (struct joint_data_msg *)&g_sys.joints[i];
        rt_kprintf("  J%d     %s %s %s %s %s  %s\n",
                   jd->joint_id,
                   ftoa(jd->angle, 2), ftoa(jd->speed, 2),
                   ftoa(jd->torque, 2), ftoa(jd->voltage, 2),
                   ftoa(jd->current, 2),
                   g_sys.joint_online[i] ? "ON" : "OFF");
    }

    rt_kprintf("[SYS] Torque Baseline: [");
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        rt_kprintf("%s%s", ftoa(g_sys.torque_baseline[i], 2),
                   i < MAX_JOINT_COUNT - 1 ? ", " : "");
    rt_kprintf("] Nm\n");

    rt_kprintf("[SYS] Pi Monitor: RAW=%s  Stream=%s@%dms\n",
               g_raw_monitor ? "ON" : "OFF",
               g_stream_enabled ? "ON" : "OFF",
               g_stream_rate_ms);
    rt_kprintf("\n");
}

/* ---- info <id> ---- */
static void cmd_info(void)
{
    _usage = "info <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    if (id < 1 || id > 6) { rt_kprintf("  Invalid joint ID (1~6)\n"); return; }

    /* 当前数据 */
    int idx = id - 1;
    struct joint_data_msg *jd = (struct joint_data_msg *)&g_sys.joints[idx];

    rt_kprintf("\n  Joint %d:\n", id);
    rt_kprintf("    Online:    %s\n", g_sys.joint_online[idx] ? "YES" : "NO");
    rt_kprintf("    Angle:     %s °\n", ftoa(jd->angle, 2));
    rt_kprintf("    Speed:     %s r/min\n", ftoa(jd->speed, 2));
    rt_kprintf("    Torque:    %s Nm\n", ftoa(jd->torque, 2));
    rt_kprintf("    Voltage:   %s V\n", ftoa(jd->voltage, 2));
    rt_kprintf("    Current:   %s A\n", ftoa(jd->current, 2));
    rt_kprintf("    Zero OK:   %s\n", g_sys.zero_valid[idx] ? "YES" : "NO");

    /* 实时回读 PID — 先关自动流避免帧冲突 */
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    disable_angle_speed_torque_state(id);
    rt_thread_mdelay(20);
    struct PID pid = get_pid(id);
    /* 回读限位范围 */
    float amin = read_property(id, 38004, 0);
    float amax = read_property(id, 38005, 0);
    enable_angle_speed_torque_state(id);
    set_state_feedback_rate_ms(id, JOINT_FEEDBACK_RATE_MS);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("    PID:       P=%s I=%s D=%s\n", ftoa(pid.P, 2), ftoa(pid.I, 2), ftoa(pid.D, 2));
    rt_kprintf("    Limit:     [%s, %s] °\n", ftoa(amin, 1), ftoa(amax, 1));

    rt_kprintf("\n");
}

/* ---- j<id> [angle] [speed] ---- */
static void cmd_joint(const char *cmd_str)
{
    int id = atoi(cmd_str + 1);  /* skip 'j' */
    if (id < 1 || id > 6) {
        rt_kprintf("  Invalid joint ID (1~6). Usage: j<id> [angle] [speed]\n");
        return;
    }

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);

    if (argc == 0) {
        /* 读取: 直接从 g_sys 读模型角度 (不打断自动流, 不干扰 joint_data_thread) */
        int idx = id - 1;
        rt_kprintf("  J%d: angle=%s°  speed=%sr/min  torque=%sNm  [online=%s]\n",
                   id,
                   ftoa(g_sys.joints[idx].angle, 2),
                   ftoa(g_sys.joints[idx].speed, 2),
                   ftoa(g_sys.joints[idx].torque, 2),
                   g_sys.joint_online[idx] ? "YES" : "NO");
    } else {
        /* 写入：只发 CAN 帧, 不需要 RX 锁 */
        float angle = (float)atof(argv[0]);
        float speed = (argc >= 2) ? (float)atof(argv[1]) : 30.0f;
        if (_positive_block(id, angle)) {
            rt_mutex_release(mutex_can_tx);
            return;
        }
        set_angle(id, angle, speed, 10.0f, 1);
        rt_kprintf("  J%d -> %s° (speed=%sr/min)\n", id, ftoa(angle, 1), ftoa(speed, 1));
    }

    rt_mutex_release(mutex_can_tx);
}

/* ---- m <j1> <j2> <j3> <j4> ---- */
static void cmd_multi(void)
{
    _usage = "m <j1> <j2> <j3> <j4> <j5> <j6> [speed]";
    NEED_ARGS(6);
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    float angles[6];
    for (int i = 0; i < 6; i++)
        angles[i] = (float)atof(argv[i]);
    float speed = (argc >= 7) ? (float)atof(argv[6]) : 60.0f;

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    /* 安全拦截: J2/J3/J4 正值检查 */
    for (int i = 0; i < 6; i++) {
        if (_positive_block(ids[i], angles[i])) {
            rt_mutex_release(mutex_can_tx);
            return;
        }
    }
    set_angles(ids, angles, speed, 100.0f, 1, 6);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  MOVJ: [%s, %s, %s, %s, %s, %s] @ %sr/min\n",
               ftoa(angles[0], 1), ftoa(angles[1], 1), ftoa(angles[2], 1),
               ftoa(angles[3], 1), ftoa(angles[4], 1), ftoa(angles[5], 1),
               ftoa(speed, 0));
}

/* ---- g <angle> [servo_id] / g cola <0|1> ---- */
static void cmd_gripper(void)
{
    NEED_ARGS(1);

    /* g cola: 可乐夹爪专用模式 */
    if (strcmp(argv[0], "cola") == 0) {
        NEED_ARGS(2);
        int mode = atoi(argv[1]);
        float angle;
        if (mode == 0) {
            angle = 20.0f;   /* 松开释放 */
        } else if (mode == 1) {
            angle = 90.0f;   /* 闭合夹取 */
        } else {
            rt_kprintf("  g cola <0|1>  0=release(20°) 1=grip(90°)\n");
            return;
        }
        int sid = (argc >= 3) ? atoi(argv[2]) : 3;  /* 默认 SERVO_3 */
        if (sid < 1 || sid > 5) { rt_kprintf("  Servo ID 1~5\n"); return; }
        Servo_SetAngle((ServoID)(sid - 1), angle);
        rt_kprintf("  Gripper %d -> %s° (cola %s)\n", sid, ftoa(angle, 1),
                   mode == 0 ? "release" : "grip");
        return;
    }

    /* 通用: g <angle> [servo_id] */
    float angle = (float)atof(argv[0]);
    int sid = (argc >= 2) ? atoi(argv[1]) : 3;  /* 默认 SERVO_3 */
    if (sid < 1 || sid > 5) { rt_kprintf("  Servo ID 1~5\n"); return; }
    Servo_SetAngle((ServoID)(sid - 1), angle);
    rt_kprintf("  Gripper %d -> %s°\n", sid, ftoa(angle, 1));
}

/* ---- e / estop [id] ---- */
static void cmd_estop(void)
{
    if (argc >= 1) {
        int id = atoi(argv[0]);
        estop(id);
        rt_kprintf("  ESTOP joint %d\n", id);
    } else {
        estop(0);  /* broadcast */
        rt_kprintf("  ESTOP ALL\n");
    }
    g_sys.estop_triggered = 1;
    g_sys.state = SYS_STATE_ESTOP;
}

/* ---- z / zero ---- */
static void cmd_zero(void)
{
    uint8_t ids[MAX_JOINT_COUNT] = {1, 2, 3, 4, 5, 6};
    float zeros[MAX_JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    set_angles(ids, zeros, 40.0f, 50.0f, 1, MAX_JOINT_COUNT);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  HOME: all joints to 0°\n");
}

/* ---- ready ---- */
static void cmd_ready(void)
{
    uint8_t ids[MAX_JOINT_COUNT] = {1, 2, 3, 4, 5, 6};
    float ready_angles[MAX_JOINT_COUNT] = {
        0.0f,    /* J1 */
        -50.0f,  /* J2 */
        -55.0f,  /* J3 */
        -70.0f,  /* J4 */
        110.0f,  /* J5 */
        0.0f     /* J6 */
    };
    g_sys.state = SYS_STATE_READY;
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(30000));
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (_positive_block(ids[i], ready_angles[i])) continue;
        rt_kprintf("  READY: J%d → %.2f° ...\n", ids[i], ready_angles[i]);
        set_angle(ids[i], ready_angles[i], 15.0f, 50.0f, 1);
        position_done(ids[i]);
        rt_thread_mdelay(1000);
    }
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    g_sys.state = SYS_STATE_RUNNING;
    rt_kprintf("  READY: all joints at calibrated positions\n");
}

/* ---- home ----
 * home     : 反向 J5→J4→J3→J2→J1→J6 (ready 回收路径, 防撞机)
 * home 1   : 正向 J1→J2→J3→J4→J5→J6 (零位部署) */
static void cmd_home(void)
{
    uint8_t ids[6];
    float   home_angles[6];
    const char *mode_str;

    int forward = (argc >= 1 && strcmp(argv[0], "1") == 0);
    if (forward) {
        /* 正向: 零位→home */
        uint8_t fwd[6] = {1, 2, 3, 4, 5, 6};
        float   ang[6] = {0.0f, -9.95f, -10.0f, -10.0f, 0.0f, 0.0f};
        memcpy(ids, fwd, sizeof(ids));
        memcpy(home_angles, ang, sizeof(home_angles));
        mode_str = "HOME(fwd)";
    } else {
        /* 反向: ready→home 回收 */
        uint8_t rev[6] = {5, 4, 3, 2, 1, 6};
        float   ang[6] = {0.0f, -10.0f, -10.0f, -9.95f, 0.0f, 0.0f};
        memcpy(ids, rev, sizeof(ids));
        memcpy(home_angles, ang, sizeof(home_angles));
        mode_str = "HOME(rev)";
    }

    g_sys.state = SYS_STATE_READY;
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(30000));
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (_positive_block(ids[i], home_angles[i])) continue;
        rt_kprintf("  %s: J%d → %.2f° ...\n", mode_str, ids[i], home_angles[i]);
        set_angle(ids[i], home_angles[i], 15.0f, 50.0f, 1);
        position_done(ids[i]);
        rt_thread_mdelay(1000);
    }
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    g_sys.state = SYS_STATE_RUNNING;
    rt_kprintf("  HOME: all joints at home calibration positions\n");
}

/* ---- cal <id> ---- */
static void cmd_cal(void)
{
    _usage = "cal <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    if (id < 1 || id > 6) { rt_kprintf("  Invalid joint ID\n"); return; }

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    motion_aid(id, 0.0f, 30.0f, 0.3f, 0.5f, 0.8f);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  Calibration: J%d force-assist ON. Drag to mechanical zero.\n", id);
    rt_kprintf("  Then send 'calz %d' to confirm.\n", id);
}

/* ---- calz <id> ---- */
static void cmd_calz(void)
{
    _usage = "calz <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    if (id < 1 || id > 6) return;

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    disable_angle_speed_torque_state(id);
    rt_thread_mdelay(20);
    set_zero_position(id);
    save_config(id);
    rt_thread_mdelay(150);
    float angle = get_angle(id);
    enable_angle_speed_torque_state(id);
    set_state_feedback_rate_ms(id, JOINT_FEEDBACK_RATE_MS);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);

    if (fabsf(angle) < 1.0f) {
        g_sys.zero_valid[id - 1] = 1;
        rt_kprintf("  J%d zero set OK (verify: %s°)\n", id, ftoa(angle, 2));
    } else {
        rt_kprintf("  J%d zero set FAIL! verify angle=%s°\n", id, ftoa(angle, 2));
    }
}

/* ---- cal_end ---- */
static void cmd_cal_end(void)
{
    rt_kprintf("  Calibration end.\n");
    g_sys.state = SYS_STATE_RUNNING;
}

/* ---- setid <old> <new> ---- */
static void cmd_setid(void)
{
    _usage = "setid <old_id> <new_id>";
    NEED_ARGS(2);
    int old_id = atoi(argv[0]);
    int new_id = atoi(argv[1]);
    if (new_id < 1 || new_id > 64) { rt_kprintf("  ID range 1~64\n"); return; }

    rt_kprintf("  Setting ID %d -> %d ... (single joint on bus required!)\n", old_id, new_id);
    disable_angle_speed_torque_state(old_id);
    rt_thread_mdelay(20);
    set_id(old_id, new_id);
    rt_thread_mdelay(800);

    uint8_t verify = get_id(0);
    enable_angle_speed_torque_state(new_id);
    set_state_feedback_rate_ms(new_id, JOINT_FEEDBACK_RATE_MS);
    rt_kprintf("  Verify: %d  %s\n", verify, (verify == new_id) ? "OK!" : "FAIL!");
}

/* ---- setzero <id> ---- */
static void cmd_setzero(void)
{
    _usage = "setzero <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    if (id < 1 || id > 6) return;

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    set_zero_position(id);
    save_config(id);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d zero set (permanent)\n", id);
}

/* ---- pid <id> <p> <i> <d> ---- */
static void cmd_pid(void)
{
    _usage = "pid <id> <p> <i> <d>";
    NEED_ARGS(4);
    int id = atoi(argv[0]);
    float p = (float)atof(argv[1]);
    float i = (float)atof(argv[2]);
    float d = (float)atof(argv[3]);

    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    set_pid(id, p, i, d);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d PID: P=%s I=%s D=%s\n", id, ftoa(p, 2), ftoa(i, 2), ftoa(d, 2));
}

/* ---- reboot <id> ---- */
static void cmd_reboot(void)
{
    _usage = "reboot <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    reboot(id);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d rebooting...\n", id);
}

/* ---- save <id> ---- */
static void cmd_save(void)
{
    _usage = "save <id>";
    NEED_ARGS(1);
    int id = atoi(argv[0]);
    save_config(id);
    rt_kprintf("  J%d config saved to flash\n", id);
}

/* ---- speed <id> <speed> ---- */
static void cmd_speed(void)
{
    _usage = "speed <id> <speed>";
    NEED_ARGS(2);
    int id = atoi(argv[0]);
    float spd = (float)atof(argv[1]);
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    set_speed_limit(id, spd);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d speed limit = %s r/min\n", id, ftoa(spd, 1));
}

/* ---- torque <id> <torque> ---- */
static void cmd_torque(void)
{
    _usage = "torque <id> <torque>";
    NEED_ARGS(2);
    int id = atoi(argv[0]);
    float tq = (float)atof(argv[1]);
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    set_torque_limit(id, tq);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d torque limit = %s Nm\n", id, ftoa(tq, 2));
}

/* ---- mode <id> <1|2> ---- */
static void cmd_mode(void)
{
    _usage = "mode <id> <1=idle|2=closed>";
    NEED_ARGS(2);
    int id = atoi(argv[0]);
    int mode = atoi(argv[1]);
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(5000));
    set_mode(id, mode);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    rt_kprintf("  J%d mode = %d (%s)\n", id, mode,
               mode == 1 ? "idle" : mode == 2 ? "closed-loop" : "?");
}

/* ---- clear ---- */
static void cmd_clear(void)
{
    g_sys.error_code = ERR_NONE;
    g_sys.estop_triggered = 0;
    safety_clear_faults();
    memset(g_safety_can_tick, 0, sizeof(g_safety_can_tick));
    if (g_sys.state == SYS_STATE_ESTOP || g_sys.state == SYS_STATE_ERROR)
        g_sys.state = SYS_STATE_RUNNING;
    clear_error(0);
    rt_kprintf("  ESTOP & error cleared, system resumed\n");
}

/* ---- raw on/off ---- */
static void cmd_raw(void)
{
    _usage = "raw on|off";
    NEED_ARGS(1);
    if (strcmp(argv[0], "on") == 0) {
        g_raw_monitor = 1;
        rt_kprintf("  Pi RAW monitor: ON\n");
    } else {
        g_raw_monitor = 0;
        rt_kprintf("  Pi RAW monitor: OFF\n");
    }
}

/* ---- pi on/off ---- */
static void cmd_pi(void)
{
    _usage = "pi on|off";
    NEED_ARGS(1);
    if (strcmp(argv[0], "on") == 0) {
        g_stream_enabled = 1;
        g_stream_rate_ms = 20;
        rt_kprintf("  Pi passthrough: ON @%dms\n", g_stream_rate_ms);
    } else {
        g_stream_enabled = 0;
        rt_kprintf("  Pi passthrough: OFF\n");
    }
}

/* ---- stream <rate_ms>  ---- */
static void cmd_stream(void)
{
    _usage = "stream <rate_ms>";
    NEED_ARGS(1);
    int rate = atoi(argv[0]);
    if (rate <= 0) {
        g_stream_enabled = 0;
        rt_kprintf("  Telemetry stream: OFF\n");
    } else {
        g_stream_rate_ms = rate;
        g_stream_enabled = 1;
        rt_kprintf("  Telemetry stream: ON @%dms\n", rate);
    }
}

/* ---- getid ---- */
static void cmd_getid(void)
{
    /* 关所有关节自动流 */
    for (int i = 1; i <= 6; i++) disable_angle_speed_torque_state(i);
    rt_thread_mdelay(20);
    uint8_t id = get_id(0);
    for (int i = 1; i <= 6; i++) {
        enable_angle_speed_torque_state(i);
        set_state_feedback_rate_ms(i, JOINT_FEEDBACK_RATE_MS);
    }
    rt_kprintf("  Bus ID: %d (single joint on bus!)\n", id);
}

/* ---- canmon on|off ---- */
static void cmd_canmon(void)
{
    _usage = "canmon on|off";
    NEED_ARGS(1);
    if (strcmp(argv[0], "on") == 0) {
        g_can_monitor = 1;
        g_cm_rx_flag = 0;
        rt_kprintf("  CAN bus monitor: ON (raw frames)\n");
    } else {
        g_can_monitor = 0;
        rt_kprintf("  CAN bus monitor: OFF\n");
    }
}

/* ---- cantest 1|0|raw ---- */
static void cmd_cantest(void)
{
    _usage = "cantest 1|0|raw";
    NEED_ARGS(1);
    if (strcmp(argv[0], "raw") == 0) {
        /* 裸寄存器诊断 */
        rt_kprintf("\n=== CAN1 Register Dump ===\n");
        rt_kprintf("MCR=0x%08X  MSR=0x%08X  TSR=0x%08X\n",
                   CAN1->MCR, CAN1->MSR, CAN1->TSR);
        rt_kprintf("ESR=0x%08X  BTR=0x%08X  RF0R=0x%08X\n",
                   CAN1->ESR, CAN1->BTR, CAN1->RF0R);
        rt_kprintf("IER=0x%08X\n", CAN1->IER);

        /* 错误计数 */
        uint32_t esr = CAN1->ESR;
        rt_kprintf("  TEC=%lu (TX err cnt)  REC=%lu (RX err cnt)\n",
                   (esr >> 16) & 0xFF, (esr >> 24) & 0xFF);
        rt_kprintf("  Bus-Off=%d  Err-Passive=%d  LastErr=%lu\n",
                   (esr & CAN_ESR_BOFF) ? 1 : 0,
                   (esr & CAN_ESR_EPVF) ? 1 : 0,
                   (esr & CAN_ESR_LEC) >> 4);

        /* GPIO 状态 */
        rt_kprintf("PB8(IDR)=%d  PB9(IDR)=%d\n",
                   (GPIOB->IDR >> 8) & 1, (GPIOB->IDR >> 9) & 1);

        /* 尝试裸发送一帧 */
        rt_kprintf("Attempting bare TX: ");
        if (CAN1->TSR & CAN_TSR_TME0) {
            CAN1->sTxMailBox[0].TIR = 0x1801AA;  /* EXT ID */
            CAN1->sTxMailBox[0].TDTR = 1;         /* DLC=1 */
            CAN1->sTxMailBox[0].TDLR = 0x42;      /* data='B' */
            CAN1->sTxMailBox[0].TDHR = 0;
            CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
            rt_kprintf("TXRQ sent. TSR=0x%08X\n", CAN1->TSR);
        } else {
            rt_kprintf("Mailbox0 busy! TSR=0x%08X\n", CAN1->TSR);
        }
        rt_kprintf("\n");
    } else if (strcmp(argv[0], "1") == 0) {
        can_loopback_enable();
        g_can_test = 1;
        g_can_test_seq = 0;
        rt_kprintf("\n  CAN Loopback Test: ON (internal LB)\n");
        rt_kprintf("  ID=0x1801AA  DLC=1  period=1000ms  data=seq++\n");
        rt_kprintf("  TX and RX shown on console\n\n");
    } else {
        g_can_test = 0;
        can_loopback_disable();
        rt_kprintf("  CAN Test: OFF\n");
    }
}

/* ======================================================================== */
/*   命令分发                                                                */
/* ======================================================================== */

static void dispatch(const char *cmd)
{
    /* ---- 帮助 ---- */
    if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0) {
        cmd_help();
        return;
    }
    /* ---- 状态 ---- */
    if (strcmp(cmd, "s") == 0 || strcmp(cmd, "status") == 0) {
        cmd_status();
        return;
    }
    /* ---- 急停 ---- */
    if (strcmp(cmd, "e") == 0 || strcmp(cmd, "estop") == 0) {
        cmd_estop();
        return;
    }
    /* ---- 归零 ---- */
    if (strcmp(cmd, "z") == 0 || strcmp(cmd, "zero") == 0) {
        cmd_zero();
        return;
    }
    /* ---- ready ---- */
    if (strcmp(cmd, "ready") == 0) {
        cmd_ready();
        return;
    }
    /* ---- home ---- */
    if (strcmp(cmd, "home") == 0) {
        cmd_home();
        return;
    }
    /* ---- 清除错误 ---- */
    if (strcmp(cmd, "clear") == 0) {
        cmd_clear();
        return;
    }
    /* ---- 读取ID ---- */
    if (strcmp(cmd, "getid") == 0) {
        cmd_getid();
        return;
    }
    /* ---- cal_end ---- */
    if (strcmp(cmd, "cal_end") == 0) {
        cmd_cal_end();
        return;
    }

    /* ---- 带参数命令 ---- */
    if (cmd[0] == 'j' && cmd[1] >= '0' && cmd[1] <= '9') {
        cmd_joint(cmd);
        return;
    }
    if (strcmp(cmd, "m") == 0)    { cmd_multi();    return; }
    if (strcmp(cmd, "g") == 0)    { cmd_gripper();  return; }
    if (strcmp(cmd, "info") == 0) { cmd_info();     return; }
    if (strcmp(cmd, "cal") == 0)  { cmd_cal();      return; }
    if (strcmp(cmd, "calz") == 0) { cmd_calz();     return; }
    if (strcmp(cmd, "setid") == 0) { cmd_setid();   return; }
    if (strcmp(cmd, "setzero") == 0) { cmd_setzero(); return; }
    if (strcmp(cmd, "pid") == 0)  { cmd_pid();      return; }
    if (strcmp(cmd, "reboot") == 0) { cmd_reboot(); return; }
    if (strcmp(cmd, "save") == 0) { cmd_save();     return; }
    if (strcmp(cmd, "speed") == 0) { cmd_speed();   return; }
    if (strcmp(cmd, "torque") == 0) { cmd_torque(); return; }
    if (strcmp(cmd, "mode") == 0) { cmd_mode();     return; }
    if (strcmp(cmd, "raw") == 0)  { cmd_raw();      return; }
    if (strcmp(cmd, "pi") == 0)   { cmd_pi();       return; }
    if (strcmp(cmd, "stream") == 0) { cmd_stream(); return; }
    if (strcmp(cmd, "canmon") == 0)  { cmd_canmon();  return; }
    if (strcmp(cmd, "cantest") == 0) { cmd_cantest(); return; }

    rt_kprintf("  Unknown command: '%s'. Type 'h' for help.\n", cmd);
}

/* ======================================================================== */
/*   行解析                                                                  */
/* ======================================================================== */
static void parse_line(void)
{
    /* 拷贝到可写缓冲区 */
    memcpy(argv_buf, cmd_buf, cmd_pos + 1);
    argv_buf[cmd_pos] = '\0';

    /* 用空格切分 */
    argc = 0;
    char *token = strtok(argv_buf, " \t");
    while (token && argc < CMD_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }

    if (argc == 0) return;  /* 空行 */

    /* 第一个参数是命令名, 后续是参数 */
    const char *cmd_name = argv[0];
    /* 把 argc/argv 偏移一下: 去掉命令名 */
    argc--;
    for (int i = 0; i < argc; i++)
        argv[i] = argv[i + 1];

    dispatch(cmd_name);
}

/* ======================================================================== */
/*   调试命令行线程                                                          */
/* ======================================================================== */
static rt_uint8_t  debug_stack[1024];
static struct rt_thread debug_tcb;

static void debug_thread_entry(void *parameter)
{
    Dbg_Uart_StartRx();

    rt_kprintf("\n[Debug] Console ready. Type 'h' for help.\n");
    rt_kprintf("[Debug] > ");

    while (1)
    {
        while (Dbg_Uart_Available()) {
            uint8_t ch = (uint8_t)Dbg_Uart_GetChar();

            if (ch == '\r' || ch == '\n') {
                /* 执行命令 */
                if (cmd_pos > 0) {
                    rt_kprintf("\n");
                    parse_line();
                    cmd_pos = 0;
                }
                rt_kprintf("[Debug] > ");

            } else if (ch == 0x08 || ch == 0x7F) {
                /* Backspace */
                if (cmd_pos > 0) {
                    cmd_pos--;
                    rt_kprintf("\b \b");
                }

            } else if (ch >= 0x20 && ch < 0x7F) {
                /* 可打印字符 */
                if (cmd_pos < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_pos++] = (char)ch;
                    rt_kprintf("%c", ch);
                }
            }
        }

        /* ---- CAN 回环测试: cantest 1 ---- */
        /* ---- CAN RX 回环打印 (从 ISR 安全缓冲区读) ---- */
        if (g_lb_rx_flag) {
            rt_kprintf("[CAN RX] IDE=%s ID=0x%08X DLC=%d data=",
                       g_lb_rx_ide ? "EXT" : "STD", g_lb_rx_id, g_lb_rx_dlc);
            for (int i = 0; i < g_lb_rx_dlc; i++)
                rt_kprintf("%02X ", g_lb_rx_data[i]);
            rt_kprintf("\n");
            g_lb_rx_flag = 0;
        }

        /* ---- CAN 总线监听打印 (canmon on) ---- */
        if (g_cm_rx_flag) {
            rt_kprintf("[CAN] IDE=%s ID=0x%03X DLC=%d data=",
                       g_cm_rx_ide ? "EXT" : "STD", g_cm_rx_id, g_cm_rx_dlc);
            for (int i = 0; i < g_cm_rx_dlc; i++)
                rt_kprintf("%02X ", g_cm_rx_data[i]);
            rt_kprintf("\n");
            g_cm_rx_flag = 0;
        }

        /* ---- CAN 回环测试发送: cantest 1 ---- */
        if (g_can_test) {
            static uint32_t last_can_tick = 0;
            uint32_t now = rt_tick_get();
            if (now - last_can_tick >= rt_tick_from_millisecond(1000)) {
                last_can_tick = now;
                uint8_t d[1] = { g_can_test_seq };
                uint8_t ret = Can_Send_Msg(0x1801AA, 1, d);
                rt_kprintf("[CAN TX] ID=0x1801AA data=%02X ret=%d\n", g_can_test_seq, ret);
                g_can_test_seq++;
            }
        }

        /* ---- 周期状态打印: pi on 时每 2s 输出一次摘要 ---- */
        {
            static uint32_t last_summary = 0;
            if (g_stream_enabled &&
                (rt_tick_get() - last_summary) > rt_tick_from_millisecond(2000)) {
                last_summary = rt_tick_get();
                rt_kprintf("[Pi] J=[%s %s %s %s %s %s]°  State=%d  Err=%d\n",
                           ftoa(g_sys.joints[0].angle, 1),
                           ftoa(g_sys.joints[1].angle, 1),
                           ftoa(g_sys.joints[2].angle, 1),
                           ftoa(g_sys.joints[3].angle, 1),
                           ftoa(g_sys.joints[4].angle, 1),
                           ftoa(g_sys.joints[5].angle, 1),
                           g_sys.state, g_sys.error_code);
            }
        }

        rt_thread_mdelay(20);
    }
}

/* ======================================================================== */
/*   初始化                                                                  */
/* ======================================================================== */
int debug_cmd_init(void)
{
    rt_err_t result = rt_thread_init(&debug_tcb, "debug",
                                      debug_thread_entry,
                                      RT_NULL,
                                      debug_stack,
                                      sizeof(debug_stack),
                                      4,      /* priority  (仅高于 LED) */
                                      5);
    if (result == RT_EOK) {
        rt_thread_startup(&debug_tcb);
        return RT_EOK;
    }
    return -RT_ERROR;
}
