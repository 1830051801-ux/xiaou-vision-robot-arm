/**
 ******************************************************************************
 * @file    app_threads.c
 * @brief   4轴机械臂 — RT-Thread 多线程架构实现 (6线程, 执行器模式)
 * @date    2026-07-27 (架构重构: 算法层迁移至 ROS2)
 ******************************************************************************
 * 线程架构 (6线程):
 *   P=2  watchdog_thread     (512B)   IWDG feed + thread heartbeat monitor
 *   P=5  joint_data_thread   (2048B)  CAN joint data RX (4 joints)
 *   P=6  safety_thread       (1024B)  Joint limit + overcurrent + ESTOP
 *   P=8  joint_ctrl_thread   (2048B)  Trajectory point executor + zero check + calib
 *   P=15 comm_thread         (1536B)  UART protocol parsing + command dispatch
 *   P=22 led_thread          (512B)   LED status indication
 *
 * 数据流:
 *   CAN_IRQ → joint_data → mb_joint_data → safety (安全检查)
 *                                  |
 *   UART_RX → comm → mb_ctrl_cmd → joint_ctrl → CAN_TX (轨迹点执行)
 *                       mb_telemetry ← joint_ctrl (关节状态回传)
 *
 * 已移除:
 *   - arm_kinematics (FK/IK/Jacobian/重力补偿) → ROS2
 *   - trajectory_planner (S曲线/笛卡尔直线) → ROS2
 *   - grasp_sequence (三段式抓取) → ROS2
 *   - base_offset 管理 → ROS2 TF2
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "app_threads.h"
#include "can.h"
#include "iwdg.h"
#include "gpio.h"
#include "usart.h"
#include "led.h"
#include "watchdog.h"
#include "comm_protocol.h"
#include "calibration.h"
#include "calib_defaults.h"
#include "trajectory.h"       /* v1.2: 轨迹 FIFO + 校验 + 到位检测 */
#include "safety.h"           /* v1.2: 安全策略 + 故障锁存 */
#include "debug_cmd.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 全局对象                                                                   */
/* -------------------------------------------------------------------------- */

/* 线程句柄 (6线程) */
rt_thread_t led_thread         = RT_NULL;
rt_thread_t joint_data_thread  = RT_NULL;
rt_thread_t joint_ctrl_thread  = RT_NULL;
rt_thread_t safety_thread      = RT_NULL;
rt_thread_t comm_thread        = RT_NULL;
rt_thread_t watchdog_thread    = RT_NULL;

/* 系统状态 */
struct sys_status g_sys;

/* 信号量 */
rt_sem_t sem_joint_data_ready  = RT_NULL;
rt_sem_t sem_uart_rx_done      = RT_NULL;
rt_sem_t sem_can_rx_done       = RT_NULL;

/* 互斥量 */
rt_mutex_t mutex_sys_state     = RT_NULL;
rt_mutex_t mutex_can_tx        = RT_NULL;
rt_mutex_t mutex_can_rx        = RT_NULL;

/* v1.4: 安全线程 CAN 时间戳 — 全局可见，clear 时复位 */
rt_tick_t g_safety_can_tick[MAX_JOINT_COUNT] = {0};

/* v2: 轨迹 Quintic 插值状态 — joint_ctrl + trajectory 模块共享 */
struct traj_interp_state g_interp;

/* 邮箱 */
rt_mailbox_t mb_ctrl_cmd       = RT_NULL;
rt_mailbox_t mb_joint_data     = RT_NULL;
rt_mailbox_t mb_telemetry      = RT_NULL;

/* 协议解析器 (comm 线程独占) */
static struct proto_parser g_proto;
/* 标定上下文 (joint_ctrl + joint_data + comm 线程使用) */
struct calib_ctx g_calib;

/* 关节限位常量 (编译期确定, safety_thread + trajectory 使用) */
const float g_joint_angle_min[MAX_JOINT_COUNT] = JOINT_ANGLE_MIN_LIST;
const float g_joint_angle_max[MAX_JOINT_COUNT] = JOINT_ANGLE_MAX_LIST;
const float g_joint_speed_max[MAX_JOINT_COUNT] = JOINT_SPEED_MAX_LIST;
const float g_joint_torque_max[MAX_JOINT_COUNT] = JOINT_TORQUE_MAX_LIST;

/* 遥测流控制 (调试命令行可读写, 故非 static) */
volatile int g_stream_enabled = 0;
volatile int g_stream_rate_ms = 50;

/* -------------------------------------------------------------------------- */
/* 1. LED 指示灯线程   P=22  stack=512B   period=50ms                        */
/* -------------------------------------------------------------------------- */
void led_thread_entry(void *parameter)
{
    LED_Init();

    while (1)
    {
        led_show_status(g_sys.state);
        Watchdog_Heartbeat(HB_IDX_LED);
        rt_thread_mdelay(50);
    }
}

/* -------------------------------------------------------------------------- */
/* 2. 关节数据接收线程   P=5   stack=2048B  period=5ms                       */
/* -------------------------------------------------------------------------- */
void joint_data_thread_entry(void *parameter)
{
    struct joint_data_msg msg;

    /* Pi v1.2: ISR 驱动在线判定 — g_last_feedback_tick[] 由 CAN ISR 更新 (can.h) */
    #define ONLINE_TIMEOUT_TICK   rt_tick_from_millisecond(50)
    #define RETRY_INTERVAL_TICK   rt_tick_from_millisecond(2000)  /* 离线关节每2秒重试启用反馈 */

    /* 离线关节反馈流重试跟踪 (解决关节后上电场景: MCU 先启动, 关节后上电) */
    rt_tick_t last_retry_tick[MAX_JOINT_COUNT] = {0};

    rt_kprintf("[JointData] thread started (ISR-driven online, 50ms timeout)\n");

    /* 启用所有关节的实时状态流 (不存在的关节会失败, 由在线检测处理) */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        uint8_t id = CAN_ID_JOINT_1 + i;
        rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
        rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(1000));
        enable_angle_speed_torque_state(id);
        set_state_feedback_rate_ms(id, JOINT_FEEDBACK_RATE_MS);
        rt_mutex_release(mutex_can_rx);
        rt_mutex_release(mutex_can_tx);
        rt_thread_mdelay(2);
    }
    rt_kprintf("[JointData] streaming config done for %d joints\n", MAX_JOINT_COUNT);

    rt_thread_mdelay(100);

    while (1)
    {
        rt_tick_t now = rt_tick_get();

        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            uint8_t id = CAN_ID_JOINT_1 + i;

            /* ── Pi v1.3: ISR 驱动在线判定 (每关节新鲜时间戳) ──
             * rt_tick_get() 必须在此处重新获取: for 循环遍历 6 关节
             * 可能跨越多帧 (每次 fresh_feedback 最多 20ms), 循环开头的
             * now 已经过期, 用过期 now 做减法会导致 g_last_feedback_tick
             * 大于 now → 无符号下溢 → 显示 4294967295ms 并误判离线。 */
            rt_tick_t now_j = rt_tick_get();
            int is_online = ((now_j - g_last_feedback_tick[i]) <= ONLINE_TIMEOUT_TICK);

            if (!is_online) {
                if (g_sys.joint_online[i]) {
                    if (g_last_feedback_tick[i] == 0) {
                        rt_kprintf("[JointData] J%d offline (no feedback since boot)\n", id);
                    } else {
                        rt_kprintf("[JointData] J%d offline (last fb %lums ago)\n",
                                   id, (unsigned long)(now_j - g_last_feedback_tick[i]));
                    }
                }
                rt_mutex_take(mutex_sys_state, RT_WAITING_FOREVER);
                g_sys.joint_online[i] = 0;
                rt_mutex_release(mutex_sys_state);

                /* v1.2: 定期重试启用反馈流 — 解决"关节后上电"场景
                 * 若 MCU 启动时关节未上电, 初始 enable 会静默失败,
                 * 此后关节永远没有反馈流 → g_last_feedback_tick 不更新 → 永远离线。
                 * 这里每 2s 重试一次 enable, 直到 ISR 真正收到反馈帧。 */
                if ((now - last_retry_tick[i]) > RETRY_INTERVAL_TICK) {
                    last_retry_tick[i] = now;
                    if (rt_mutex_take(mutex_can_tx, rt_tick_from_millisecond(50)) == RT_EOK) {
                        if (rt_mutex_take(mutex_can_rx, rt_tick_from_millisecond(200)) == RT_EOK) {
                            enable_angle_speed_torque_state(id);
                            set_state_feedback_rate_ms(id, JOINT_FEEDBACK_RATE_MS);
                            rt_mutex_release(mutex_can_rx);
                        }
                        rt_mutex_release(mutex_can_tx);
                    }
                }
                continue;  /* 不发起 CAN 读取, 避免阻塞 */
            }

            /* ── v1.3: ISR 驱动的按关节新鲜反馈数据 ──
             * CAN ISR 已做 CAN ID 匹配 + DLC==8 过滤, 将反馈帧路由到 g_raw_fb[i]。
             * 此处零 CAN 通信直接读取 — 避免 6 关节顺序调用
             * angle_speed_torque_state_fresh() 导致的信号量耗尽。 */
            if (!g_raw_fb[i].fresh) continue;  /* 本轮无本轴新反馈帧, 下轮再查 */

            /* 从 ISR 预存缓冲区解析反馈帧:
             *   [0..3] = angle  (float32 LE)
             *   [4..5] = speed  (int16 LE, x0.01)
             *   [6..7] = torque (int16 LE, x0.01) */
            struct angle_speed_torque ast = {0.0f, 0.0f, 0.0f};
            memcpy(&ast.angle, &g_raw_fb[i].data[0], 4);
            int16_t s16;
            memcpy(&s16, &g_raw_fb[i].data[4], 2);
            ast.speed = s16 * 0.01f;
            memcpy(&s16, &g_raw_fb[i].data[6], 2);
            ast.torque = s16 * 0.01f;
            g_raw_fb[i].fresh = 0;  /* 消费数据 */

            /* ── 构建关节数据消息 ── */
            msg.joint_id  = id;
            /* v1.2: 原始 CAN 反馈 → 模型角度 (决策 A: MCU 负责变换) */
            msg.angle     = calib_raw_to_model(&g_calib, id, ast.angle);
            msg.speed     = ast.speed;
            msg.torque    = ast.torque;
            msg.voltage   = 0.0f;
            msg.current   = 0.0f;
            msg.timestamp = now;

            /* v1.3: 移除实时流中的 get_volcur() — 电压/电流查询会
             * 中断 CAN 反馈帧时序, 单关节阻塞读取无法跟上 6 轴轮询 */

            /* ── 更新全局快照 (CMD_GET_JOINT 只读此快照, 不阻塞 CAN) ── */
            rt_mutex_take(mutex_sys_state, RT_WAITING_FOREVER);
            memcpy(&g_sys.joints[i], &msg, sizeof(msg));
            /* online 由 ISR timestamp 唯一决定: 仅当刚收到该关节 CAN 帧时置 1 */
            g_sys.joint_online[i] = 1;
            rt_mutex_release(mutex_sys_state);

            /* 发送到安全线程 (不阻塞, 邮箱满则丢弃) */
            rt_mb_send_wait(mb_joint_data, (rt_ubase_t)&msg, 0);
        }
        g_sys.system_tick = now;

        Watchdog_Heartbeat(HB_IDX_JOINT_DATA);
        rt_thread_mdelay(5);
    }
}

/* -------------------------------------------------------------------------- */
/* 3. 安全保护线程   P=6   stack=1024B  period=10ms                          */
/* -------------------------------------------------------------------------- */
void safety_thread_entry(void *parameter)
{
    rt_err_t            result;
    rt_ubase_t          msg_val;
    struct joint_data_msg *joint;

    rt_kprintf("[Safety] thread started\n");

    while (1)
    {
        result = rt_mb_recv(mb_joint_data, &msg_val, 10);

        if (result == RT_EOK) {
            joint = (struct joint_data_msg *)msg_val;

            if (joint->joint_id < 1 || joint->joint_id > MAX_JOINT_COUNT)
                goto next_cycle;

            int idx = joint->joint_id - 1;
            /* 始终更新 CAN 时间戳，避免 clear 后误报超时 */
            g_safety_can_tick[idx] = rt_tick_get();

            if (!g_sys.estop_triggered) {
                /* ---- 预测性限位检查 ---- */
                float angle_pred = joint->angle + joint->speed * 0.06f;
                float range = g_joint_angle_max[idx] - g_joint_angle_min[idx];
                float margin = range * (1.0f - ARM_PREDICT_LIMIT_RATIO) * 0.5f;
                float soft_min = g_joint_angle_min[idx] + margin;
                float soft_max = g_joint_angle_max[idx] - margin;

                /* 硬预测: 下一周期将超限 → 急停 */
                if (angle_pred < g_joint_angle_min[idx] || angle_pred > g_joint_angle_max[idx]) {
                    rt_kprintf("[Safety] JOINT%d predictive ESTOP! pred=%d°\n",
                               joint->joint_id, (int)(angle_pred + 0.5f));
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_LIMIT_TRIGGERED;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }

                /* 软保护: 降速 */
                if (angle_pred < soft_min || angle_pred > soft_max) {
                    float reduced_speed = g_joint_speed_max[idx] * 0.5f;
                    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                    set_speed_limit(joint->joint_id, reduced_speed);
                    rt_mutex_release(mutex_can_tx);
                }

                /* ---- 关节角度限位 ---- */
                if (joint->angle < g_joint_angle_min[idx] ||
                    joint->angle > g_joint_angle_max[idx]) {
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_LIMIT_TRIGGERED;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }

                /* ---- 转速超限 ---- */
                if (fabsf(joint->speed) > g_joint_speed_max[idx] * 1.1f) {
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_LIMIT_TRIGGERED;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }

                /* ---- 力矩过载 ---- */
                if (fabsf(joint->torque) > g_joint_torque_max[idx] * 1.2f) {
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_JOINT_OVERCURRENT;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }

                /* ---- 电流异常 ---- */
                if (joint->current > ARM_OVERCURRENT_THRESHOLD) {
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_JOINT_OVERCURRENT;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }
            }
        }

    next_cycle:
        /* ---- CAN 通信超时检查 ---- */
        if (!g_sys.estop_triggered) {
            rt_tick_t now = rt_tick_get();
            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                if (g_safety_can_tick[i] == 0) continue;
                if ((now - g_safety_can_tick[i]) > rt_tick_from_millisecond(ARM_CAN_TIMEOUT_MS)) {
                    rt_kprintf("[Safety] JOINT%d CAN timeout! (last=%dms)\n",
                               i + 1, (int)(now - g_safety_can_tick[i]));
                    estop(CAN_ID_JOINT_1 + i);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_JOINT_COMM_LOSS;
                    g_sys.state = SYS_STATE_ESTOP;
                }
            }
        }

        Watchdog_Heartbeat(HB_IDX_SAFETY);
        rt_thread_mdelay(10);
    }
}

/* -------------------------------------------------------------------------- */
/* 4. 关节控制线程   P=8   stack=2048B  period=10ms (执行器模式)              */
/* -------------------------------------------------------------------------- */
void joint_ctrl_thread_entry(void *parameter)
{
    rt_err_t            result;
    rt_ubase_t          msg_val;
    struct ctrl_msg    *cmd;
    struct telemetry_msg telem;
    float               current_joints[MAX_JOINT_COUNT] = {0};
    float               traj_target_model[MAX_JOINT_COUNT];
    uint8_t             joint_ids[MAX_JOINT_COUNT];
    uint16_t            current_traj_duration = 0;
    rt_tick_t           last_telem_tick = 0;
    int                 traj_safety_block = 0;

    rt_kprintf("[JointCtrl] thread started (executor mode)\n");

    /* 初始化关节 ID 列表 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        joint_ids[i] = CAN_ID_JOINT_1 + i;
    }

    /* 等待关节数据就绪 */
    rt_thread_mdelay(500);

    /* ==================================================================== */
    /*  上电零点校验 (v1.2: 仅当 Flash 有标定数据时才校验)                  */
    /* ==================================================================== */
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        current_joints[i] = g_sys.joints[i].angle;

    calib_init(&g_calib, CALIB_ZERO_DRIFT_DEG);

    if (g_calib.state == CALIB_ZERO_OK) {
        /* Flash 有有效标定数据 → 必须通过零点校验才能运行 */
        g_sys.state = SYS_STATE_ZERO_CHECK;
        rt_kprintf("[JointCtrl] Flash calib valid — running zero check "
                   "(threshold=%.2f°)\n", CALIB_ZERO_DRIFT_DEG);

    {
        int check_done = 0;
        int check_rounds = 0;
        uint32_t check_deadline = rt_tick_get()
                                + rt_tick_from_millisecond(CALIB_CHECK_TIMEOUT_MS + 1000);
        while (!check_done && rt_tick_get() < check_deadline) {
            rt_thread_mdelay(50);
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                current_joints[i] = g_sys.joints[i].angle;

            check_rounds++;
            if (check_rounds < CALIB_CHECK_MIN_ROUNDS)
                continue;

            /* v1.2: 只校验在线关节 — 离线关节不参与判定 */
            int online_count = 0;
            int all_online_ok = 1;
            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                if (!g_sys.joint_online[i]) continue;
                online_count++;
                if (fabsf(current_joints[i]) <= CALIB_ZERO_DRIFT_DEG) {
                    g_calib.zero_valid[i] = 1;
                } else {
                    g_calib.zero_valid[i] = 0;
                    all_online_ok = 0;
                }
            }

            if (online_count == 0) continue;  /* 没有关节在线, 继续等待 */

            if (all_online_ok) {
                g_calib.state = CALIB_ZERO_OK;
                check_done = 1;
            }
        }

        if (!check_done) {
            /* 超时: 有在线关节未通过校验 */
            rt_kprintf("[JointCtrl] *** ZERO CHECK FAILED! ***\n");
            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                if (g_sys.joint_online[i]) {
                    rt_kprintf("[JointCtrl]   J%d angle=%.2f° %s\n",
                               i + 1, current_joints[i],
                               g_calib.zero_valid[i] ? "OK" : "FAIL");
                }
            }
            g_calib.state = CALIB_ZERO_LOST;
            g_sys.error_code = ERR_ZERO_LOST;
            g_sys.state = SYS_STATE_ERROR;
        } else {
            float torques[MAX_JOINT_COUNT];
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                torques[i] = g_sys.joints[i].torque;
            calib_record_torque_baseline(&g_calib, torques);
            rt_kprintf("[JointCtrl] Zero check PASSED (%d online joints). "
                       "Torque baseline: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] Nm\n",
                       check_rounds > 0 ? 1 : 0,  /* at least J1 */
                       torques[0], torques[1], torques[2],
                       torques[3], torques[4], torques[5]);
        }

        /* 同步标定状态到全局 */
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            g_sys.zero_valid[i] = g_calib.zero_valid[i];
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            g_sys.torque_baseline[i] = g_calib.torque_baseline[i];
    }
    } else {
        /* Flash 无标定数据 — 首次上电, 跳过零点校验, 直接进入运行 */
        rt_kprintf("[JointCtrl] No calib data in Flash — skipping zero check, "
                   "system RUNNING (uncalibrated)\n");
        g_calib.state = CALIB_ZERO_LOST;  /* 未标定, 但不锁死系统 */
        /* 离线关节标记为 unchecked, 在线关节标记为待标定 */
        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            g_calib.zero_valid[i] = 0;
            g_sys.zero_valid[i]   = 0;
        }
        g_sys.error_code = ERR_NOT_CALIBRATED;  /* 非致命, 仅提示未标定 */
    }

    if (g_sys.state != SYS_STATE_ERROR) {
        g_sys.state = SYS_STATE_RUNNING;
        rt_kprintf("[JointCtrl] System RUNNING (executor mode)\n");
    } else {
        rt_kprintf("[JointCtrl] System LOCKED (zero lost) -- awaiting calibration\n");
    }

    while (1)
    {
        /* 阻塞等待控制指令 */
        result = rt_mb_recv(mb_ctrl_cmd, &msg_val, TRAJ_INTERPOLATION_MS);
        cmd = (struct ctrl_msg *)msg_val;

        /* ---- 急停拦截 ---- */
        if (g_sys.estop_triggered) {
            if (result == RT_EOK && cmd->msg_type == CTRL_MSG_ESTOP) {
                estop(0);
                g_interp.active = 0;
                traj_fifo_clear();
                g_sys.motion_busy = 0;
            }
            Watchdog_Heartbeat(HB_IDX_JOINT_CTRL);
            rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
            continue;
        }

        /* ---- 零点丢失拦截 ---- */
        if (g_sys.error_code == ERR_ZERO_LOST) {
            if (result == RT_EOK) {
                switch (cmd->msg_type) {
                case CTRL_MSG_CALIB_START:
                case CTRL_MSG_CALIB_END:
                case CTRL_MSG_SET_ZERO:
                case CTRL_MSG_ESTOP:
                case CTRL_MSG_REBOOT:
                case CTRL_MSG_QUERY_STATE:
                    break;
                default:
                    Watchdog_Heartbeat(HB_IDX_JOINT_CTRL);
                    rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
                    continue;
                }
            }
        }

        /* ---- 命令分发 ---- */
        if (result == RT_EOK) {
            switch (cmd->msg_type) {

            /* ===== 轨迹点已由 comm 线程直接入队 (traj_enqueue_single), 此处不再处理 ===== */
            case CTRL_MSG_TRAJ_POINT:
                /* v1.2: comm 线程直接写入 FIFO, joint_ctrl 从 FIFO 出队执行 */
                break;

            case CTRL_MSG_TRAJ_CLEAR:
                traj_fifo_clear();
                g_interp.active = 0;
                g_sys.motion_busy = 0;
                rt_kprintf("[JointCtrl] Trajectory buffer cleared\n");
                break;

            /* ===== 停止 ===== */
            case CTRL_MSG_STOP:
                g_interp.active = 0;
                traj_fifo_clear();
                g_sys.motion_busy = 0;
                estop(0);
                rt_kprintf("[JointCtrl] STOP\n");
                break;

            /* ===== 单关节角度 (调试保留) ===== */
            case CTRL_MSG_ANGLE: {
                /* v1.2: 模型度 → 原始度 */
                float raw_angle = calib_model_to_raw(&g_calib, cmd->joint_id, cmd->param1);
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                set_angle(cmd->joint_id, raw_angle, cmd->param2,
                          cmd->param3, cmd->mode);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;
            }

            /* ===== 急停 ===== */
            case CTRL_MSG_ESTOP:
                estop(0);
                g_interp.active = 0;
                traj_fifo_clear();
                safety_trigger_estop();
                g_sys.estop_triggered = 1;
                g_sys.state = SYS_STATE_ESTOP;
                g_sys.motion_busy = 0;
                rt_kprintf("[JointCtrl] ESTOP triggered\n");
                break;

            /* ===== 清除错误 ===== */
            case CTRL_MSG_CLEAR_ERROR:
                g_sys.error_code = ERR_NONE;
                g_sys.estop_triggered = 0;
                safety_clear_faults();
                if (g_sys.state == SYS_STATE_ESTOP || g_sys.state == SYS_STATE_ERROR)
                    g_sys.state = SYS_STATE_RUNNING;
                break;

            /* ===== 设置零点 ===== */
            case CTRL_MSG_SET_ZERO: {
                int zero_ret;
                uint8_t jid = cmd->joint_id;
                if (g_calib.state == CALIB_CALIBRATING) {
                    /* v1.2: calib_confirm_zero 需要当前原始反馈 */
                    /* 从 g_sys 读取模型角度, 反算原始角度 */
                    float model_angle = g_sys.joints[jid - 1].angle;
                    float raw_angle = calib_model_to_raw(&g_calib, jid, model_angle);
                    zero_ret = calib_confirm_zero(&g_calib, jid, raw_angle);
                    if (zero_ret >= 0) {
                        g_sys.zero_valid[jid - 1] = 1;
                        rt_kprintf("[JointCtrl] CALIB SET_ZERO J%d OK "
                                   "(model=%.2f°, raw=%.2f°)\n",
                                   jid, model_angle, raw_angle);
                        if (g_sys.error_code == ERR_ZERO_LOST) {
                            /* 检查是否全部恢复 */
                            int all_ok = 1;
                            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                                if (!g_calib.zero_valid[i]) { all_ok = 0; break; }
                            }
                            if (all_ok) {
                                g_sys.error_code = ERR_NONE;
                                g_sys.state = SYS_STATE_RUNNING;
                                rt_kprintf("[JointCtrl] All zeros recovered!\n");
                            }
                        }
                    } else {
                        rt_kprintf("[JointCtrl] CALIB SET_ZERO J%d FAIL "
                                   "(model=%.2f°, raw=%.2f°)\n",
                                   jid, model_angle, raw_angle);
                    }
                } else {
                    /* 非标定模式: 直接 set_zero_position */
                    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                    rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                    set_zero_position(jid);
                    rt_mutex_release(mutex_can_rx);
                    rt_mutex_release(mutex_can_tx);
                    rt_kprintf("[JointCtrl] SET_ZERO joint=%d\n", jid);
                }
                break;
            }

            /* ===== 标定 ===== */
            case CTRL_MSG_CALIB_START:
                if (calib_start_joint(&g_calib, cmd->joint_id) == 0) {
                    g_sys.state = SYS_STATE_CALIBRATION;
                    rt_kprintf("[JointCtrl] CALIB_START J%d: force-assist enabled\n",
                               cmd->joint_id);
                }
                break;

            case CTRL_MSG_CALIB_END:
                calib_end(&g_calib);
                g_sys.state = (g_calib.state == CALIB_ZERO_OK)
                              ? SYS_STATE_RUNNING : SYS_STATE_ERROR;
                rt_kprintf("[JointCtrl] CALIB_END: state=%d\n", g_calib.state);
                break;

            /* ===== 夹爪控制 ===== */
            case CTRL_MSG_GRIPPER:
                Servo_SetAngle((ServoID)(cmd->joint_id), cmd->param1);
                break;

            /* ===== 参数设置 ===== */
            case CTRL_MSG_SET_PID:
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                set_pid(cmd->joint_id, cmd->param1, cmd->param2, cmd->param3);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;

            case CTRL_MSG_SET_MODE:
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                set_mode(cmd->joint_id, (int)cmd->param1);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;

            case CTRL_MSG_REBOOT:
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                reboot(cmd->joint_id);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;

            /* ===== v1.2: 保存标定到 Flash ===== */
            case CTRL_MSG_SAVE_CONFIG: {
                int save_ret = calib_save_to_flash(&g_calib);
                if (save_ret == 0) {
                    rt_kprintf("[JointCtrl] SAVE_CONFIG: Flash write OK\n");
                } else {
                    rt_kprintf("[JointCtrl] SAVE_CONFIG: Flash write FAIL (err=%d)\n",
                               save_ret);
                }
                break;
            }

            default:
                break;
            }
        }

        /* ---- 轨迹 Quintic 插值执行 (v2: 10ms 每周期, mode=0 轨迹跟踪) ---- */
        if (!g_sys.estop_triggered && g_sys.error_code != ERR_ZERO_LOST) {

            /* 状态 0: 空闲 → 检查 FIFO → 启动新段 */
            if (!g_interp.active && traj_fifo_count() > 0) {
                int dq_ret = traj_dequeue(traj_target_model, &current_traj_duration);
                if (dq_ret == 0) {
                    /* 安全检查 */
                    traj_safety_block = safety_check_motion(traj_target_model);
                    if (traj_safety_block != SAFETY_OK) {
                        rt_kprintf("[JointCtrl] SAFETY BLOCK: motion denied "
                                   "(fault=0x%02X)\n", traj_safety_block);
                        g_sys.motion_busy = 0;
                        goto skip_traj_exec;
                    }

                    /* 初始化 Quintic 插值状态 */
                    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                        g_interp.q_start[i]  = g_sys.joints[i].angle;
                        g_interp.q_target[i] = traj_target_model[i];
                        g_interp.joint_skip[i] = (uint8_t)(
                            fabsf(g_interp.q_target[i] - g_interp.q_start[i]) < 0.02f);
                    }
                    g_interp.duration_ms = current_traj_duration;
                    g_interp.elapsed_ms  = 0;
                    g_interp.active      = 1;
                    g_sys.motion_busy    = 1;
                }
            }

            /* 状态 1: 插值进行中 */
            if (g_interp.active) {
                /* 超时保护: 3x planned duration, 最少 3 秒 */
                uint32_t timeout_ms = g_interp.duration_ms * 3;
                if (timeout_ms < 3000) timeout_ms = 3000;

                if (g_interp.elapsed_ms >= timeout_ms) {
                    rt_kprintf("[JointCtrl] WARN: interp timeout (%dms > %dms), "
                               "forcing next\n",
                               g_interp.elapsed_ms, timeout_ms);
                    /* 强制到位: 跳过当前段, 尝试出队下一段 */
                    goto traj_next_segment;
                }

                /* 每 10ms tick: 计算插值 + 发送 CAN */
                int tick_ret = traj_interp_tick(&g_interp);
                if (tick_ret == 1) {
                    /* 段到位 → 尝试出队下一段 */
                    traj_next_segment:
                    if (traj_fifo_count() > 0) {
                        int dq_ret = traj_dequeue(traj_target_model,
                                                  &current_traj_duration);
                        if (dq_ret == 0) {
                            traj_safety_block = safety_check_motion(traj_target_model);
                            if (traj_safety_block != SAFETY_OK) {
                                rt_kprintf("[JointCtrl] SAFETY BLOCK mid-trajectory "
                                           "(fault=0x%02X), clearing FIFO\n",
                                           traj_safety_block);
                                traj_fifo_clear();
                                g_interp.active   = 0;
                                g_sys.motion_busy = 0;
                                goto skip_traj_exec;
                            }

                            /* 链式启动: 从上一段终点开始 */
                            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                                g_interp.q_start[i]  = g_interp.q_target[i];
                                g_interp.q_target[i] = traj_target_model[i];
                                g_interp.joint_skip[i] = (uint8_t)(
                                    fabsf(g_interp.q_target[i] - g_interp.q_start[i]) < 0.02f);
                            }
                            g_interp.duration_ms = current_traj_duration;
                            g_interp.elapsed_ms  = 0;
                            g_interp.active      = 1;  /* 链式继续 */
                        }
                    } else {
                        g_interp.active   = 0;
                        g_sys.motion_busy = 0;
                    }
                } else if (tick_ret < 0) {
                    /* 插值错误: 中止 */
                    rt_kprintf("[JointCtrl] Interpolation error, aborting "
                               "trajectory\n");
                    traj_fifo_clear();
                    g_interp.active   = 0;
                    g_sys.motion_busy = 0;
                    g_sys.error_code  = ERR_TRAJ_ERROR;
                }
            }
        }
        skip_traj_exec: ;

        /* ---- 更新当前关节角度 ---- */
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            current_joints[i] = g_sys.joints[i].angle;

        /* ---- 遥测数据发布 ---- */
        if (g_stream_enabled &&
            (rt_tick_get() - last_telem_tick) >= rt_tick_from_millisecond(g_stream_rate_ms)) {
            telem.timestamp = rt_tick_get();
            telem.sys_state = g_sys.state;
            telem.error_code = g_sys.error_code;
            telem.motion_busy = g_sys.motion_busy;
            telem.estop_triggered = g_sys.estop_triggered;
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                memcpy(&telem.joints[i], &g_sys.joints[i], sizeof(struct joint_data_msg));

            rt_mb_send(mb_telemetry, (rt_ubase_t)&telem);
            last_telem_tick = rt_tick_get();
        }

        Watchdog_Heartbeat(HB_IDX_JOINT_CTRL);
        rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
    }
}

/* -------------------------------------------------------------------------- */
/* 5. 通讯线程   P=15  stack=1536B  period=20ms                               */
/* -------------------------------------------------------------------------- */
void comm_thread_entry(void *parameter)
{
    struct proto_frame frame;
    struct ctrl_msg    cmd;
    struct telemetry_msg telem;
    rt_err_t           result;
    rt_ubase_t         msg_val;
    uint8_t            tx_buf[PROTO_TX_BUF_SIZE];
    uint16_t           tx_len;
    uint8_t            seq = 0;

    rt_kprintf("[Comm] thread started\n");

    proto_init(&g_proto);

    while (1)
    {
        /* ---- UART 接收: 从 USART1 读入字节到协议解析器 ---- */
        while (Rpi_Uart_Available()) {
            uint8_t ch = (uint8_t)Rpi_Uart_GetChar();
            if (g_raw_monitor) {
                rt_kprintf("[Pi RAW] 0x%02X '%c'\n", ch,
                          (ch >= 0x20 && ch < 0x7F) ? ch : '.');
            }
            proto_rx_byte(&g_proto, ch);
        }

        /* ---- 尝试解析帧 ---- */
        if (proto_try_parse(&g_proto, &frame)) {
            rt_kprintf("[Comm] RX frame: cmd=0x%02X len=%d seq=%d\n",
                       frame.cmd, frame.len, frame.seq);

            memset(&cmd, 0, sizeof(cmd));

#if ARM_ACTUATOR_OUTPUTS_ENABLED == 0
            /* ---- transport-only: 锁定运动命令, 仅响应状态查询 ---- */
            {
                uint8_t motion_locked = 0;
                switch (frame.cmd) {
                    case CMD_ESTOP: case CMD_STOP:
                    case CMD_TRAJ_POINT: case CMD_TRAJ_BATCH:
                    case CMD_TRAJ_BUFFER_CLEAR: case CMD_GRIPPER:
                    case CMD_SET_ZERO: case CMD_CALIB_START:
                    case CMD_CALIB_END: case CMD_SET_PID:
                    case CMD_REBOOT_JOINT: case CMD_SAVE_CONFIG:
                        proto_build_response(RSP_MOTION_LOCKED, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        Rpi_Uart_Send(tx_buf, tx_len);
                        motion_locked = 1;
                        break;
                }
                if (motion_locked) continue;  /* 跳过主 switch, 读下一帧 */
            }
#endif

            switch (frame.cmd) {

            /* ---- 系统控制 ---- */
            case CMD_PING:
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_ESTOP:
                /* v1.2: 清空 FIFO (mutex 保护) + 通知 joint_ctrl 停止 CAN */
                traj_fifo_clear();
                cmd.msg_type = CTRL_MSG_ESTOP;
                cmd.joint_id = 0;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_CLEAR_ERROR:
                cmd.msg_type = CTRL_MSG_CLEAR_ERROR;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_STOP:
                /* v1.2: 清空 FIFO + 通知 joint_ctrl 停止 CAN */
                traj_fifo_clear();
                cmd.msg_type = CTRL_MSG_STOP;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            /* ---- 轨迹点下发 (6关节, 26B) [Pi v1.2: 使用轨迹模块] ---- */
            case CMD_TRAJ_POINT:
                if (frame.len == 26) {
                    if (g_sys.estop_triggered) {
                        proto_build_response(RSP_ESTOP_ACTIVE, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        break;
                    }
                    if (g_sys.state == SYS_STATE_CALIBRATION ||
                        g_sys.state == SYS_STATE_ZERO_CHECK ||
                        g_sys.state == SYS_STATE_INIT) {
                        proto_build_response(RSP_BUSY, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        break;
                    }
                    /* v1.2: 直接入队 (mutex 保护), 不再经过 mailbox */
                    int err = traj_enqueue_single(frame.payload, 0);
                    switch (err) {
                    case TRAJ_OK: {
                        uint8_t free_slot[1] = { traj_fifo_free() };
                        proto_build_response(RSP_TRAJ_ACK, frame.seq,
                                           free_slot, 1, tx_buf, &tx_len);
                        break;
                    }
                    case TRAJ_ERR_FIFO_FULL:
                        proto_build_response(RSP_TRAJ_OVERFLOW, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        break;
                    default: {
                        uint8_t inv[1] = { frame.cmd };
                        proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                           inv, 1, tx_buf, &tx_len);
                        break;
                    }
                    }
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            /* ---- 批量轨迹点下发 (N x 26B) [Pi v1.2: 原子入队, 使用轨迹模块] ---- */
            case CMD_TRAJ_BATCH:
                if (frame.len >= 26 && (frame.len % 26) == 0) {
                    uint8_t n = frame.len / 26;
                    uint8_t reject_code = 0;  /* 0=ok, 1=lock, 2=estop, 3=busy, 4=overflow */

                    if (n < 1 || n > TRAJ_BATCH_MAX_POINTS)
                        reject_code = 1;

                    if (reject_code == 0 && g_sys.estop_triggered)
                        reject_code = 2;
                    if (reject_code == 0 &&
                        (g_sys.state == SYS_STATE_CALIBRATION ||
                         g_sys.state == SYS_STATE_ZERO_CHECK ||
                         g_sys.state == SYS_STATE_INIT))
                        reject_code = 3;

                    if (reject_code == 0) {
                        /* v1.2: traj_enqueue_batch 内部做两遍校验 + 原子写入 */
                        int batch_ret = traj_enqueue_batch(frame.payload, n, 0);
                        if (batch_ret < 0) {
                            /* 校验或空间不足 */
                            if (batch_ret == TRAJ_ERR_FIFO_FULL)
                                reject_code = 4;
                            else
                                reject_code = 1;
                        } else {
                            /* 成功: 返回入队数量 */
                            uint8_t free_slot[1] = { traj_fifo_free() };
                            proto_build_response(RSP_TRAJ_ACK, frame.seq,
                                               free_slot, 1, tx_buf, &tx_len);
                            rt_kprintf("[Comm] BATCH: %d pts enqueued, free=%d\n",
                                      batch_ret, traj_fifo_free());
                        }
                    }

                    if (reject_code != 0) {
                        { uint8_t inv[1] = { frame.cmd };
                        switch (reject_code) {
                            case 1:
                                proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                                   inv, 1, tx_buf, &tx_len);
                                break;
                            case 2:
                                proto_build_response(RSP_ESTOP_ACTIVE, frame.seq,
                                                   NULL, 0, tx_buf, &tx_len);
                                break;
                            case 3:
                                proto_build_response(RSP_BUSY, frame.seq,
                                                   NULL, 0, tx_buf, &tx_len);
                                break;
                            case 4:
                                proto_build_response(RSP_TRAJ_OVERFLOW, frame.seq,
                                                   NULL, 0, tx_buf, &tx_len);
                                break;
                        }}
                    }
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_TRAJ_BUFFER_CLEAR:
                /* v1.2: 同时清空 FIFO + 通知 joint_ctrl */
                traj_fifo_clear();
                cmd.msg_type = CTRL_MSG_TRAJ_CLEAR;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                {
                    uint8_t free_slot[1] = { TRAJ_FIFO_CAPACITY };
                    proto_build_response(RSP_TRAJ_ACK, frame.seq,
                                       free_slot, 1, tx_buf, &tx_len);
                }
                break;

            /* ---- 夹爪控制 [Pi v1.2: 使用 trajectory 模块的有限性校验] ---- */
            case CMD_GRIPPER:
                if (frame.len == 5) {
                    float grip_angle;
                    memcpy(&grip_angle, &frame.payload[1], 4);
                    if (!traj_is_finite(grip_angle)) {
                        { uint8_t inv[1] = { frame.cmd };
                          proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                             inv, 1, tx_buf, &tx_len); }
                        break;
                    }
                    cmd.msg_type = CTRL_MSG_GRIPPER;
                    cmd.joint_id = frame.payload[0];
                    cmd.param1   = grip_angle;
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            /* ---- 状态查询 (精简: 不含末端位姿) ---- */
            case CMD_GET_STATE: {
                uint8_t state_payload[4 + MAX_JOINT_COUNT * 13];  /* 4头+6×(4+4+4+1)=82B */
                int off = 0;
                state_payload[off++] = g_sys.state;
                state_payload[off++] = g_sys.error_code;
                state_payload[off++] = g_sys.estop_triggered;
                state_payload[off++] = g_sys.motion_busy;
                for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                    memcpy(&state_payload[off], (void *)&g_sys.joints[i].angle, 4);  off += 4;
                    memcpy(&state_payload[off], (void *)&g_sys.joints[i].speed, 4);  off += 4;
                    memcpy(&state_payload[off], (void *)&g_sys.joints[i].torque, 4); off += 4;
                    state_payload[off++] = g_sys.joint_online[i];
                }
                proto_build_response(RSP_ACK, frame.seq, state_payload, off, tx_buf, &tx_len);
                break;
            }

            /* ---- 单关节状态查询 [Pi v1.2: 14B payload] ---- */
            case CMD_GET_JOINT:
                if (frame.len == 1) {
                    uint8_t jid = frame.payload[0];
                    if (jid >= 1 && jid <= MAX_JOINT_COUNT) {
                        /* v1.2: joint_id(1) + angle(4) + speed(4) + torque(4) + online(1) = 14B */
                        uint8_t jp[14];
                        int jo = 0;
                        jp[jo++] = jid;
                        memcpy(&jp[jo], (void *)&g_sys.joints[jid - 1].angle, 4);  jo += 4;
                        memcpy(&jp[jo], (void *)&g_sys.joints[jid - 1].speed, 4);  jo += 4;
                        memcpy(&jp[jo], (void *)&g_sys.joints[jid - 1].torque, 4); jo += 4;
                        jp[jo++] = g_sys.joint_online[jid - 1];
                        proto_build_response(RSP_ACK, frame.seq, jp, jo, tx_buf, &tx_len);
                    } else {
                        { uint8_t inv[1] = { frame.cmd };
                          proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                             inv, 1, tx_buf, &tx_len); }
                    }
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_STREAM_START:
                if (frame.len >= 2) {
                    memcpy((void *)&g_stream_rate_ms, frame.payload, 2);
                    g_stream_enabled = 1;
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_STREAM_STOP:
                g_stream_enabled = 0;
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            /* ---- 标定 [Pi v1.2: 加状态校验] ---- */
            case CMD_SET_ZERO:
                if (frame.len == 1) {
                    uint8_t jid = frame.payload[0];
                    if (jid < 1 || jid > MAX_JOINT_COUNT) {
                        { uint8_t inv[1] = { frame.cmd };
                          proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                             inv, 1, tx_buf, &tx_len); }
                        break;
                    }
                    if (g_sys.estop_triggered) {
                        proto_build_response(RSP_ESTOP_ACTIVE, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        break;
                    }
                    if (g_sys.motion_busy) {
                        proto_build_response(RSP_BUSY, frame.seq,
                                           NULL, 0, tx_buf, &tx_len);
                        break;
                    }
                    if (!g_sys.joint_online[jid - 1]) {
                        { uint8_t inv[1] = { frame.cmd };
                          proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                             inv, 1, tx_buf, &tx_len); }
                        break;
                    }
                    /* 校验通过, 转 joint_ctrl 执行静止采样+读回 */
                    cmd.msg_type = CTRL_MSG_SET_ZERO;
                    cmd.joint_id = jid;
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_CALIB_START:
                if (frame.len >= 1) {
                    cmd.msg_type = CTRL_MSG_CALIB_START;
                    cmd.joint_id = frame.payload[0];
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_CALIB_END:
                cmd.msg_type = CTRL_MSG_CALIB_END;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_GET_CALIB_STATUS: {
                /* 1(state) + 6(zero_valid) + 6×4(angles) + 6×4(torque_baseline) = 55B */
                uint8_t st[1 + MAX_JOINT_COUNT + MAX_JOINT_COUNT * 4 + MAX_JOINT_COUNT * 4];
                int off = 0;
                st[off++] = g_sys.state;
                for (int i = 0; i < MAX_JOINT_COUNT; i++)
                    st[off++] = g_sys.zero_valid[i];
                for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                    memcpy(&st[off], (void *)&g_sys.joints[i].angle, 4);
                    off += 4;
                }
                for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                    memcpy(&st[off], (void *)&g_sys.torque_baseline[i], 4);
                    off += 4;
                }
                proto_build_response(RSP_ACK, frame.seq, st, off, tx_buf, &tx_len);
                break;
            }

            /* ---- 参数设置 ---- */
            case CMD_SET_PID:
                if (frame.len >= 13) {
                    cmd.msg_type = CTRL_MSG_SET_PID;
                    cmd.joint_id = frame.payload[0];
                    memcpy(&cmd.param1, &frame.payload[1], 4);
                    memcpy(&cmd.param2, &frame.payload[5], 4);
                    memcpy(&cmd.param3, &frame.payload[9], 4);
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_REBOOT_JOINT:
                if (frame.len >= 1) {
                    cmd.msg_type = CTRL_MSG_REBOOT;
                    cmd.joint_id = frame.payload[0];
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    { uint8_t inv[1] = { frame.cmd };
                      proto_build_response(RSP_INVALID_PARAM, frame.seq,
                                         inv, 1, tx_buf, &tx_len); }
                }
                break;

            case CMD_SAVE_CONFIG:
                /* v1.2: Flash 写入在 joint_ctrl 线程完成 (不在 UART 线程) */
                {
                    cmd.msg_type = CTRL_MSG_SAVE_CONFIG;
                    cmd.joint_id = 0;
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    rt_kprintf("[Comm] SAVE_CONFIG: dispatched to joint_ctrl\n");
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            default:
                proto_build_response(RSP_NACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;
            }

            /* v1.2: 发送响应帧到 USART1 */
            if (tx_len > 0) {
                Rpi_Uart_Send(tx_buf, tx_len);
                tx_len = 0;
            }
        }

        /* ---- 遥测发送 ---- */
        if (g_stream_enabled) {
            result = rt_mb_recv(mb_telemetry, &msg_val, 0);  /* non-blocking */
            if (result == RT_EOK) {
                memcpy(&telem, (void *)msg_val, sizeof(telem));
                proto_build_telemetry(seq++, (uint8_t *)&telem, sizeof(telem),
                                      tx_buf, &tx_len);
                if (tx_len > 0) {
                    Rpi_Uart_Send(tx_buf, tx_len);
                    tx_len = 0;
                }
            }
        }

        Watchdog_Heartbeat(HB_IDX_COMM);
        rt_thread_mdelay(20);
    }
}

/* -------------------------------------------------------------------------- */
/* 6. 看门狗线程   P=2   stack=512B   period=500ms                            */
/* -------------------------------------------------------------------------- */
void watchdog_thread_entry(void *parameter)
{
    int stuck_count;

    Watchdog_Init();
    rt_kprintf("[Watchdog] thread started, feed=%dms, timeout=%dms\n",
               WDT_FEED_INTERVAL_MS, WDT_TIMEOUT_MS);

    while (1)
    {
        Watchdog_Feed();
        Watchdog_Heartbeat(HB_IDX_WATCHDOG);
        stuck_count = Watchdog_CheckAll();

        if (stuck_count > 0) {
            rt_kprintf("[Watchdog] WARNING: %d thread(s) stuck!\n", stuck_count);
        }

        rt_thread_mdelay(WDT_FEED_INTERVAL_MS);
    }
}

/* -------------------------------------------------------------------------- */
/* 线程初始化                                                                 */
/* -------------------------------------------------------------------------- */
int app_threads_init(void)
{
    /* 0. 初始化系统状态 */
    rt_memset(&g_sys, 0, sizeof(g_sys));
    g_sys.state = SYS_STATE_INIT;
    /* v1.2: 预填关节号, 即使离线 status 也能显示正确的 J1~J6 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        g_sys.joints[i].joint_id = CAN_ID_JOINT_1 + i;

    /* 0.1 v1.2: 初始化轨迹 FIFO / 安全模块 / 标定上下文 (先于线程启动) */
    traj_fifo_init();
    safety_init();
    calib_init(&g_calib, CALIB_ZERO_DRIFT_DEG);
    rt_kprintf("[Init] trajectory + safety + calibration modules initialized\n");

    /* 1. 创建 IPC 对象 */

    /* 信号量 */
    sem_joint_data_ready = rt_sem_create("sem_joint", 0, RT_IPC_FLAG_FIFO);
    if (sem_joint_data_ready == RT_NULL) {
        rt_kprintf("[Init] ERROR: sem_joint_data_ready\n");
        return -RT_ERROR;
    }

    sem_uart_rx_done = rt_sem_create("sem_uart", 0, RT_IPC_FLAG_FIFO);
    if (sem_uart_rx_done == RT_NULL) {
        rt_kprintf("[Init] ERROR: sem_uart_rx_done\n");
        return -RT_ERROR;
    }

    sem_can_rx_done = rt_sem_create("sem_can_rx", 0, RT_IPC_FLAG_FIFO);
    if (sem_can_rx_done == RT_NULL) {
        rt_kprintf("[Init] ERROR: sem_can_rx_done\n");
        return -RT_ERROR;
    }

    /* 互斥量 */
    mutex_sys_state = rt_mutex_create("mutex_sys", RT_IPC_FLAG_FIFO);
    if (mutex_sys_state == RT_NULL) {
        rt_kprintf("[Init] ERROR: mutex_sys_state\n");
        return -RT_ERROR;
    }

    mutex_can_tx = rt_mutex_create("mutex_tx", RT_IPC_FLAG_FIFO);
    if (mutex_can_tx == RT_NULL) {
        rt_kprintf("[Init] ERROR: mutex_can_tx\n");
        return -RT_ERROR;
    }

    mutex_can_rx = rt_mutex_create("mutex_rx", RT_IPC_FLAG_FIFO);
    if (mutex_can_rx == RT_NULL) {
        rt_kprintf("[Init] ERROR: mutex_can_rx\n");
        return -RT_ERROR;
    }

    /* 邮箱 */
    mb_ctrl_cmd = rt_mb_create("mb_ctrl", MAILBOX_MAX_MSG, RT_IPC_FLAG_FIFO);
    if (mb_ctrl_cmd == RT_NULL) {
        rt_kprintf("[Init] ERROR: mb_ctrl_cmd\n");
        return -RT_ERROR;
    }

    mb_joint_data = rt_mb_create("mb_joint", MAILBOX_MAX_MSG, RT_IPC_FLAG_FIFO);
    if (mb_joint_data == RT_NULL) {
        rt_kprintf("[Init] ERROR: mb_joint_data\n");
        return -RT_ERROR;
    }

    mb_telemetry = rt_mb_create("mb_telem", MAILBOX_MAX_MSG, RT_IPC_FLAG_FIFO);
    if (mb_telemetry == RT_NULL) {
        rt_kprintf("[Init] ERROR: mb_telemetry\n");
        return -RT_ERROR;
    }

    rt_kprintf("[Init] IPC objects created OK (3 sem, 3 mutex, 3 mb)\n");

    /* 2. 创建 6 个线程 */

    /* 看门狗线程暂不启动 — 关节单步调试阶段不需要 */
    /* watchdog_thread = rt_thread_create("watchdog", watchdog_thread_entry,
                                       RT_NULL, 512, 2, 5);
    if (watchdog_thread != RT_NULL) rt_thread_startup(watchdog_thread); */

#if ARM_ACTUATOR_OUTPUTS_ENABLED == 1
    /* ---- 硬件使能: 启动全部 5 个线程 ---- */
    joint_data_thread = rt_thread_create("joint_data", joint_data_thread_entry,
                                         RT_NULL, 2048, 5, 5);
    if (joint_data_thread != RT_NULL) rt_thread_startup(joint_data_thread);

    safety_thread = rt_thread_create("safety", safety_thread_entry,
                                     RT_NULL, 1024, 6, 5);
    if (safety_thread != RT_NULL) rt_thread_startup(safety_thread);

    joint_ctrl_thread = rt_thread_create("joint_ctrl", joint_ctrl_thread_entry,
                                         RT_NULL, 2048, 8, 5);
    if (joint_ctrl_thread != RT_NULL) rt_thread_startup(joint_ctrl_thread);
#else
    /* ---- transport-only: 仅通讯 + LED ---- */
    rt_kprintf("[Init] TRANSPORT-ONLY: skipping joint/safety/ctrl threads\n");
#endif

    comm_thread = rt_thread_create("comm", comm_thread_entry,
                                   RT_NULL, 1536, 15, 5);
    if (comm_thread != RT_NULL) rt_thread_startup(comm_thread);

    led_thread = rt_thread_create("led", led_thread_entry,
                                  RT_NULL, 512, 19, 5);
    if (led_thread != RT_NULL) rt_thread_startup(led_thread);

    /* 3. 标记系统就绪 */
    g_sys.state = SYS_STATE_READY;
#if ARM_ACTUATOR_OUTPUTS_ENABLED == 1
    rt_kprintf("[Init] ===== 5 threads started (no watchdog), system ready =====\n");
#else
    rt_kprintf("[Init] ===== 2 threads started (transport-only), system ready =====\n");
#endif

    return RT_EOK;
}
