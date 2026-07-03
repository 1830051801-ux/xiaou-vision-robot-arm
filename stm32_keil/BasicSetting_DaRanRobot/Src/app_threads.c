/**
 ******************************************************************************
 * @file    app_threads.c
 * @brief   4轴机械臂 — RT-Thread 多线程架构实现 (6线程)
 * @date    2026-06-18 (重构)
 ******************************************************************************
 * 线程架构 (6线程):
 *   P=2  watchdog_thread     (512B)   IWDG feed, system heartbeat
 *   P=5  joint_data_thread   (2048B)  CAN joint data RX (4 joints)
 *   P=6  safety_thread       (1024B)  Joint limit, overcurrent/overtemp, ESTOP
 *   P=8  joint_ctrl_thread   (3072B)  Motion control + trajectory + kinematics
 *   P=15 comm_thread         (1536B)  UART protocol parsing + telemetry
 *   P=22 led_thread          (512B)   LED status indication
 *
 * 数据流:
 *   CAN_IRQ → joint_data → mb_joint_data → safety (安全检查)
 *                                  |
 *   UART_RX → comm → mb_ctrl_cmd → joint_ctrl → CAN_TX (运动指令)
 *                       mb_telemetry ← joint_ctrl (遥测回传)
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
#include "arm_kinematics.h"
#include "trajectory_planner.h"
#include "comm_protocol.h"
#include "calibration.h"
#include "calib_defaults.h"

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

/* 邮箱 */
rt_mailbox_t mb_ctrl_cmd       = RT_NULL;
rt_mailbox_t mb_joint_data     = RT_NULL;
rt_mailbox_t mb_telemetry      = RT_NULL;

/* 轨迹规划器 (单实例, joint_ctrl 线程独占使用) */
static struct traj_planner g_traj;
/* 协议解析器 (单实例, comm 线程独占使用) */
static struct proto_parser g_proto;
/* 标定上下文 (单实例, joint_ctrl 线程使用) */
static struct calib_ctx g_calib;

/* 关节限位常量 (编译期从 arm_config.h 确定) */
static const float g_joint_angle_min[4] = JOINT_ANGLE_MIN_LIST;
static const float g_joint_angle_max[4] = JOINT_ANGLE_MAX_LIST;
static const float g_joint_speed_max[4] = JOINT_SPEED_MAX_LIST;
static const float g_joint_torque_max[4] = JOINT_TORQUE_MAX_LIST;

/* 遥测流控制 */
static volatile int g_stream_enabled = 0;
static volatile int g_stream_rate_ms = 50;

/* -------------------------------------------------------------------------- */
/* 1. LED 指示灯线程   P=22  stack=512B   period=50ms                        */
/* -------------------------------------------------------------------------- */
void led_thread_entry(void *parameter)
{
    rt_kprintf("[LED] thread started\n");
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
    struct servo_volcur  vc;
    int volcur_tick = 0;  /* 电压/电流读取节流计数器 */

    rt_kprintf("[JointData] thread started\n");

    /* 启用所有4个关节的实时状态流 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        uint8_t id = CAN_ID_JOINT_1 + i;
        rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
        rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
        enable_angle_speed_torque_state(id);
        set_state_feedback_rate_ms(id, JOINT_FEEDBACK_RATE_MS);
        rt_mutex_release(mutex_can_rx);
        rt_mutex_release(mutex_can_tx);
        rt_thread_mdelay(2);
    }
    rt_kprintf("[JointData] streaming enabled for %d joints\n", MAX_JOINT_COUNT);

    /* 等待第一轮数据到达 */
    rt_thread_mdelay(100);

    while (1)
    {
        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            uint8_t id = CAN_ID_JOINT_1 + i;

            /* 读取实时流位置/速度/力矩 */
            rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
            struct angle_speed_torque ast = angle_speed_torque_state(id);
            rt_mutex_release(mutex_can_rx);

            msg.joint_id  = id;
            msg.angle     = ast.angle;
            msg.speed     = ast.speed;
            msg.torque    = ast.torque;
            msg.voltage   = 0.0f;
            msg.current   = 0.0f;
            msg.timestamp = rt_tick_get();

            /* 每 200ms 读取一次电压/电流 */
            if (volcur_tick == 0) {
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                vc = get_volcur(id);
                rt_mutex_release(mutex_can_rx);
                msg.voltage = vc.vol;
                msg.current = vc.cur;
            }

            /* 更新全局状态 */
            rt_mutex_take(mutex_sys_state, RT_WAITING_FOREVER);
            memcpy(&g_sys.joints[i], &msg, sizeof(msg));
            g_sys.joint_online[i] = 1;
            rt_mutex_release(mutex_sys_state);

            /* 发送到安全线程 */
            rt_mb_send(mb_joint_data, (rt_ubase_t)&msg);
        }

        /* 电压电流读取计数: 每 200ms / 5ms = 40 个周期 */
        volcur_tick = (volcur_tick + 1) % 40;

        /* 更新系统运行 tick */
        g_sys.system_tick = rt_tick_get();

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
    rt_tick_t           last_comm_tick = 0;
    rt_tick_t           last_can_tick[MAX_JOINT_COUNT] = {0};

    rt_kprintf("[Safety] thread started\n");

    while (1)
    {
        result = rt_mb_recv(mb_joint_data, &msg_val, 10);

        if (result == RT_EOK && !g_sys.estop_triggered) {
            joint = (struct joint_data_msg *)msg_val;

            if (joint->joint_id < 1 || joint->joint_id > MAX_JOINT_COUNT)
                goto next_cycle;

            int idx = joint->joint_id - 1;
            last_can_tick[idx] = rt_tick_get();

            /* ---- 预测性限位检查 (在硬限位触发前预先降速) ---- */
            if (!g_sys.estop_triggered) {
                /* 预测下一周期角度: angle_pred = angle + speed(r/min) * 0.06° */
                float angle_pred = joint->angle + joint->speed * 0.06f;
                float range = g_joint_angle_max[idx] - g_joint_angle_min[idx];
                float margin = range * (1.0f - ARM_PREDICT_LIMIT_RATIO) * 0.5f;
                float soft_min = g_joint_angle_min[idx] + margin;
                float soft_max = g_joint_angle_max[idx] - margin;

                if (angle_pred < soft_min || angle_pred > soft_max) {
                    /* 软保护: 降低转速限制到正常最大值的50% */
                    float reduced_speed = g_joint_speed_max[idx] * 0.5f;
                    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                    set_speed_limit(joint->joint_id, reduced_speed);
                    rt_mutex_release(mutex_can_tx);
                    rt_kprintf("[Safety] JOINT%d predictive limit: pred=%.2f°, reducing speed to %.0fr/min\n",
                               joint->joint_id, angle_pred, reduced_speed);
                }

                /* 硬预测: 下一个周期将超出限位 → 紧急停止 */
                if (angle_pred < g_joint_angle_min[idx] || angle_pred > g_joint_angle_max[idx]) {
                    rt_kprintf("[Safety] JOINT%d predictive ESTOP! pred=%.2f° beyond [%.2f, %.2f]\n",
                               joint->joint_id, angle_pred,
                               g_joint_angle_min[idx], g_joint_angle_max[idx]);
                    estop(joint->joint_id);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_LIMIT_TRIGGERED;
                    g_sys.state = SYS_STATE_ESTOP;
                    goto next_cycle;
                }
            }

            /* ---- 关节角度限位检查 ---- */
            if (joint->angle < g_joint_angle_min[idx] ||
                joint->angle > g_joint_angle_max[idx]) {
                rt_kprintf("[Safety] JOINT%d angle limit! angle=%.2f (min=%.2f, max=%.2f)\n",
                           joint->joint_id, joint->angle,
                           g_joint_angle_min[idx], g_joint_angle_max[idx]);
                estop(joint->joint_id);
                g_sys.estop_triggered = 1;
                g_sys.error_code = ERR_LIMIT_TRIGGERED;
                g_sys.state = SYS_STATE_ESTOP;
                goto next_cycle;
            }

            /* ---- 转速超限检查 ---- */
            if (fabsf(joint->speed) > g_joint_speed_max[idx] * 1.1f) {
                rt_kprintf("[Safety] JOINT%d speed over-limit! speed=%.2f (max=%.2f)\n",
                           joint->joint_id, joint->speed, g_joint_speed_max[idx]);
                estop(joint->joint_id);
                g_sys.estop_triggered = 1;
                g_sys.error_code = ERR_LIMIT_TRIGGERED;
                g_sys.state = SYS_STATE_ESTOP;
                goto next_cycle;
            }

            /* ---- 力矩过载检查 ---- */
            if (fabsf(joint->torque) > g_joint_torque_max[idx] * 1.2f) {
                rt_kprintf("[Safety] JOINT%d torque overload! torque=%.2f (max=%.2f)\n",
                           joint->joint_id, joint->torque, g_joint_torque_max[idx]);
                estop(joint->joint_id);
                g_sys.estop_triggered = 1;
                g_sys.error_code = ERR_JOINT_OVERCURRENT;
                g_sys.state = SYS_STATE_ESTOP;
                goto next_cycle;
            }

            /* ---- 电流异常检查 ---- */
            if (joint->current > ARM_OVERCURRENT_THRESHOLD) {
                rt_kprintf("[Safety] JOINT%d overcurrent! cur=%.2fA\n",
                           joint->joint_id, joint->current);
                estop(joint->joint_id);
                g_sys.estop_triggered = 1;
                g_sys.error_code = ERR_JOINT_OVERCURRENT;
                g_sys.state = SYS_STATE_ESTOP;
                goto next_cycle;
            }
        }

    next_cycle:
        /* ---- CAN 通信超时检查 (对所有关节) ---- */
        if (!g_sys.estop_triggered) {
            rt_tick_t now = rt_tick_get();
            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                if (g_sys.joint_online[i] &&
                    (now - last_can_tick[i]) > rt_tick_from_millisecond(ARM_CAN_TIMEOUT_MS)) {
                    rt_kprintf("[Safety] JOINT%d CAN timeout! (last=%dms ago)\n",
                               i + 1, (int)(now - last_can_tick[i]));
                    g_sys.joint_online[i] = 0;
                    estop(CAN_ID_JOINT_1 + i);
                    g_sys.estop_triggered = 1;
                    g_sys.error_code = ERR_JOINT_COMM_LOSS;
                    g_sys.state = SYS_STATE_ESTOP;
                }
            }
        }

        /* ---- 上位机通信超时 ---- */
        if (!g_sys.estop_triggered && last_comm_tick != 0 &&
            (rt_tick_get() - last_comm_tick) > rt_tick_from_millisecond(ARM_COMM_LOSS_TIMEOUT_MS)) {
            rt_kprintf("[Safety] Host communication timeout\n");
            /* 通信丢失不触发 ESTOP, 仅标记错误 */
            g_sys.error_code = ERR_HOST_COMM_LOSS;
        }

        Watchdog_Heartbeat(HB_IDX_SAFETY);
        rt_thread_mdelay(10);
    }
}

/* -------------------------------------------------------------------------- */
/* 4. 关节控制线程   P=8   stack=3072B  period=10ms                          */
/* -------------------------------------------------------------------------- */
void joint_ctrl_thread_entry(void *parameter)
{
    rt_err_t            result;
    rt_ubase_t          msg_val;
    struct ctrl_msg    *cmd;
    struct telemetry_msg telem;
    float               current_joints[4] = {0};
    float               target_joints[4];
    uint8_t             joint_ids[4];
    uint32_t            traj_start_tick = 0;
    rt_tick_t           last_telem_tick = 0;

    rt_kprintf("[JointCtrl] thread started\n");

    /* 初始化轨迹规划器 */
    traj_init(&g_traj);

    /* 等待关节数据就绪 */
    rt_thread_mdelay(500);

    /* 初始化关节 ID 列表 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        joint_ids[i] = CAN_ID_JOINT_1 + i;
        current_joints[i] = g_sys.joints[i].angle;
    }

    /* ==================================================================== */
    /*  上电零点校验                                                        */
    /* ==================================================================== */
    calib_init(&g_calib, CALIB_ZERO_DRIFT_DEG);
    calib_start_check(&g_calib);
    g_sys.state = SYS_STATE_ZERO_CHECK;
    rt_kprintf("[JointCtrl] Zero check started (threshold=%.2f°)\n",
               CALIB_ZERO_DRIFT_DEG);

    {
        int check_result = 0;
        uint32_t check_deadline = rt_tick_get()
                                + rt_tick_from_millisecond(CALIB_CHECK_TIMEOUT_MS + 1000);
        while (check_result == 0 && rt_tick_get() < check_deadline) {
            rt_thread_mdelay(50);
            /* 更新当前角度 */
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                current_joints[i] = g_sys.joints[i].angle;
            check_result = calib_check_update(&g_calib, current_joints, rt_tick_get());
        }

        if (check_result < 0) {
            /* 零点丢失: 记录 + 锁定 */
            rt_kprintf("[JointCtrl] *** ZERO LOST! Locking system. ***\n");
            for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                rt_kprintf("[JointCtrl]   J%d drift=%.2f° (threshold=%.2f°) %s\n",
                           i + 1, current_joints[i], CALIB_ZERO_DRIFT_DEG,
                           g_calib.zero_valid[i] ? "OK" : "FAIL");
            }
            g_sys.error_code = ERR_ZERO_LOST;
            g_sys.state = SYS_STATE_ERROR;
        } else {
            /* 校验通过: 记录力矩基线 */
            float torques[4];
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                torques[i] = g_sys.joints[i].torque;
            calib_record_torque_baseline(&g_calib, torques);
            rt_kprintf("[JointCtrl] Zero check PASSED.\n");
            rt_kprintf("[JointCtrl] Torque baseline: [%.2f, %.2f, %.2f, %.2f] Nm\n",
                       torques[0], torques[1], torques[2], torques[3]);
        }

        /* 同步标定状态到全局 */
        for (int i = 0; i < 3; i++)
            g_sys.base_offset[i] = g_calib.base_offset[i];
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            g_sys.zero_valid[i] = g_calib.zero_valid[i];
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            g_sys.torque_baseline[i] = g_calib.torque_baseline[i];
    }

    /* 更新 FK (应用系统原点偏移) */
    float cart_pose[6];
    arm_fk(current_joints, cart_pose);
    calib_unapply_base_offset((const float *)g_sys.base_offset, cart_pose);
    for (int i = 0; i < 6; i++) g_sys.cart_pose[i] = cart_pose[i];

    /* 根据校验结果切换最终状态 */
    if (g_sys.state != SYS_STATE_ERROR) {
        g_sys.state = SYS_STATE_RUNNING;
        rt_kprintf("[JointCtrl] System RUNNING\n");
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
                estop(0);  /* 广播急停 */
                traj_abort(&g_traj);
                g_sys.motion_busy = 0;
            }
            Watchdog_Heartbeat(HB_IDX_JOINT_CTRL);
            rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
            continue;
        }

        /* ---- 零点丢失拦截: 仅允许标定/急停/查询指令 ---- */
        if (g_sys.error_code == ERR_ZERO_LOST) {
            if (result == RT_EOK) {
                switch (cmd->msg_type) {
                case CTRL_MSG_CALIB_START:
                case CTRL_MSG_CALIB_END:
                case CTRL_MSG_SET_ZERO:
                case CTRL_MSG_ESTOP:
                case CTRL_MSG_REBOOT:
                case CTRL_MSG_SET_SYS_ORIGIN:
                case CTRL_MSG_QUERY_STATE:
                    break;  /* 允许通过 */
                default:
                    rt_kprintf("[JointCtrl] Motion denied: zero lost\n");
                    Watchdog_Heartbeat(HB_IDX_JOINT_CTRL);
                    rt_thread_mdelay(TRAJ_INTERPOLATION_MS);
                    continue;
                }
            }
        }

        /* ---- 命令分发 ---- */
        if (result == RT_EOK) {
            switch (cmd->msg_type) {

            case CTRL_MSG_MOVE_JOINTS: {
                /* 多关节 PTP 运动: param1-4 = target angles[4] */
                target_joints[0] = cmd->param1;
                target_joints[1] = cmd->param2;
                target_joints[2] = cmd->param3;
                target_joints[3] = cmd->param4;

                /* 更新当前角度 */
                for (int i = 0; i < MAX_JOINT_COUNT; i++)
                    current_joints[i] = g_sys.joints[i].angle;

                /* 配置 + 启动轨迹 */
                traj_set_joint_ptp(&g_traj, target_joints,
                    TRAJ_DEFAULT_SPEED, TRAJ_DEFAULT_ACCEL, TRAJ_DEFAULT_JERK);
                traj_start(&g_traj, rt_tick_get(), current_joints);
                g_sys.motion_busy = 1;
                traj_start_tick = rt_tick_get();
                rt_kprintf("[JointCtrl] MOVJ started\n");
                break;
            }

            case CTRL_MSG_MOVE_CARTESIAN: {
                /* 笛卡尔直线: param1-6 = target pose (Base 坐标系,
                 * 树莓派已通过手眼矩阵 T_cam2base 转换, MCU 不再做坐标变换) */
                float target_pose[6];
                target_pose[0] = cmd->param1;
                target_pose[1] = cmd->param2;
                target_pose[2] = cmd->param3;
                target_pose[3] = cmd->param4;
                target_pose[4] = cmd->param5;
                target_pose[5] = cmd->param6;

                /* 标定诊断: 应用 base_offset (正常运行时为 {0,0,0}, 无实际效果) */
                calib_apply_base_offset((const float *)g_sys.base_offset, target_pose);

                for (int i = 0; i < MAX_JOINT_COUNT; i++)
                    current_joints[i] = g_sys.joints[i].angle;

                traj_set_cart_linear(&g_traj, target_pose,
                    TRAJ_DEFAULT_CART_SPEED, TRAJ_DEFAULT_CART_ACCEL);

                /* FK 获取起始位姿 */
                arm_fk(current_joints, g_traj.start_pose);
                traj_start(&g_traj, rt_tick_get(), current_joints);
                g_sys.motion_busy = 1;
                traj_start_tick = rt_tick_get();
                rt_kprintf("[JointCtrl] MOVL target=(%.1f,%.1f,%.1f) base_offset=(%.1f,%.1f,%.1f)\n",
                           target_pose[0], target_pose[1], target_pose[2],
                           g_sys.base_offset[0], g_sys.base_offset[1], g_sys.base_offset[2]);
                break;
            }

            case CTRL_MSG_ANGLE: {
                /* 单关节角度 */
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                set_angle(cmd->joint_id, cmd->param1, cmd->param2,
                          cmd->param3, cmd->mode);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;
            }

            case CTRL_MSG_STOP:
                traj_abort(&g_traj);
                g_sys.motion_busy = 0;
                estop(0);
                rt_kprintf("[JointCtrl] STOP\n");
                break;

            case CTRL_MSG_ESTOP:
                estop(0);
                traj_abort(&g_traj);
                g_sys.estop_triggered = 1;
                g_sys.state = SYS_STATE_ESTOP;
                g_sys.motion_busy = 0;
                rt_kprintf("[JointCtrl] ESTOP triggered\n");
                break;

            case CTRL_MSG_SET_ZERO: {
                /* 标定模式下通过 calib 模块校验; 非标定模式直接设零 */
                int zero_ret;
                if (g_calib.state == CALIB_CALIBRATING) {
                    zero_ret = calib_confirm_zero(&g_calib, cmd->joint_id);
                    if (zero_ret >= 0) {
                        g_sys.zero_valid[cmd->joint_id - 1] = 1;
                        rt_kprintf("[JointCtrl] CALIB SET_ZERO J%d OK (verify passed)\n",
                                   cmd->joint_id);
                        /* 全部关节零点恢复 → 自动清除零点丢失错误 */
                        if (zero_ret == 1 && g_sys.error_code == ERR_ZERO_LOST) {
                            g_sys.error_code = ERR_NONE;
                            g_sys.state = SYS_STATE_RUNNING;
                            rt_kprintf("[JointCtrl] All zeros recovered! System resumed.\n");
                        }
                    } else {
                        rt_kprintf("[JointCtrl] CALIB SET_ZERO J%d FAIL (verify angle != 0)\n",
                                   cmd->joint_id);
                    }
                } else {
                    /* 非标定模式: 直接写入零点 */
                    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                    rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                    set_zero_position(cmd->joint_id);
                    rt_mutex_release(mutex_can_rx);
                    rt_mutex_release(mutex_can_tx);
                    rt_kprintf("[JointCtrl] SET_ZERO joint=%d\n", cmd->joint_id);
                }
                break;
            }

            case CTRL_MSG_SET_MODE:
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                set_mode(cmd->joint_id, (int)cmd->param1);
                rt_mutex_release(mutex_can_rx);
                rt_mutex_release(mutex_can_tx);
                break;

            case CTRL_MSG_IMPEDANCE:
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                impedance_control(cmd->joint_id, cmd->param1, cmd->param2,
                                  cmd->param3, cmd->param4, 0, cmd->mode);
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

            /* ---- 标定命令 ---- */
            case CTRL_MSG_CALIB_START:
                if (calib_start_joint(&g_calib, cmd->joint_id) == 0) {
                    g_sys.state = SYS_STATE_CALIBRATION;
                    rt_kprintf("[JointCtrl] CALIB_START J%d: force-assist enabled\n",
                               cmd->joint_id);
                } else {
                    rt_kprintf("[JointCtrl] CALIB_START J%d: FAILED (state=%d)\n",
                               cmd->joint_id, g_calib.state);
                }
                break;

            case CTRL_MSG_CALIB_END:
                calib_end(&g_calib);
                g_sys.state = (g_calib.state == CALIB_ZERO_OK)
                              ? SYS_STATE_RUNNING : SYS_STATE_ERROR;
                rt_kprintf("[JointCtrl] CALIB_END: state=%d\n", g_calib.state);
                break;

            case CTRL_MSG_SET_SYS_ORIGIN:
                calib_set_base_offset(&g_calib, cmd->param1, cmd->param2, cmd->param3);
                g_sys.base_offset[0] = cmd->param1;
                g_sys.base_offset[1] = cmd->param2;
                g_sys.base_offset[2] = cmd->param3;
                rt_kprintf("[JointCtrl] SET_DIAG_OFFSET: (%.1f, %.1f, %.1f) mm "
                           "(diagnostic only, should be {0,0,0} in production)\n",
                           cmd->param1, cmd->param2, cmd->param3);
                break;

            default:
                break;
            }
        }

        /* ---- 轨迹更新 (每周期执行) ---- */
        if (g_sys.motion_busy && !traj_is_done(&g_traj)) {
            uint32_t elapsed = rt_tick_get() - traj_start_tick;
            float output_joints[4];
            enum traj_state ts = traj_update(&g_traj, elapsed, output_joints);

            if (ts == TRAJ_RUNNING) {
                /* 计算重力补偿力矩 */
                float grav_torques[4];
                arm_gravity_comp(output_joints, ARM_PAYLOAD_MASS, grav_torques);

                /* 逐关节发送角度指令 + 重力前馈力矩 (mode=2: 前馈控制) */
                rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                    set_angle(joint_ids[i], output_joints[i],
                              0.0f, grav_torques[i], 2);  /* mode=2: 前馈力矩补偿重力 */
                }
                rt_mutex_release(mutex_can_tx);
            } else if (ts == TRAJ_DONE) {
                g_sys.motion_busy = 0;
                rt_kprintf("[JointCtrl] trajectory done\n");
            } else if (ts == TRAJ_ERROR) {
                g_sys.motion_busy = 0;
                rt_kprintf("[JointCtrl] trajectory ERROR!\n");
            }
        }

        /* ---- FK 更新末端位姿 (Base 坐标系, 单位为 mm/°) ---- */
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            current_joints[i] = g_sys.joints[i].angle;
        arm_fk(current_joints, cart_pose);
        /* 标定诊断: 应用 base_offset (正常运行时为 {0,0,0}, 无实际效果) */
        calib_unapply_base_offset((const float *)g_sys.base_offset, cart_pose);
        for (int i = 0; i < 6; i++) g_sys.cart_pose[i] = cart_pose[i];

        /* ---- 遥测数据发布 (按流控周期) ---- */
        if (g_stream_enabled &&
            (rt_tick_get() - last_telem_tick) >= rt_tick_from_millisecond(g_stream_rate_ms)) {
            telem.timestamp = rt_tick_get();
            telem.sys_state = g_sys.state;
            telem.error_code = g_sys.error_code;
            telem.motion_busy = g_sys.motion_busy;
            for (int i = 0; i < MAX_JOINT_COUNT; i++)
                memcpy(&telem.joints[i], &g_sys.joints[i], sizeof(struct joint_data_msg));
            memcpy(telem.cart_pose, cart_pose, sizeof(cart_pose));

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

    /* 初始化协议解析器 */
    proto_init(&g_proto);

    while (1)
    {
        /* ---- UART 接收 (从 USART1 读入数据到解析器) ---- */
        /* TODO: 实际 UART 接收 — 需要 HAL UART RX IT 或 DMA
         * 当前使用 UART 空闲中断 + RX 环形缓冲区
         * 调用 proto_rx_byte() 喂入每个接收字节 */

        /* ---- 尝试解析帧 ---- */
        if (proto_try_parse(&g_proto, &frame)) {
            rt_kprintf("[Comm] RX frame: cmd=0x%02X len=%d seq=%d\n",
                       frame.cmd, frame.len, frame.seq);

            memset(&cmd, 0, sizeof(cmd));

            switch (frame.cmd) {

            case CMD_PING:
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                /* UART 发送 tx_buf ... */
                break;

            case CMD_ESTOP:
                cmd.msg_type = CTRL_MSG_ESTOP;
                cmd.joint_id = 0;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_CLEAR_ERROR:
                g_sys.error_code = ERR_NONE;
                g_sys.estop_triggered = 0;
                if (g_sys.state == SYS_STATE_ESTOP || g_sys.state == SYS_STATE_ERROR)
                    g_sys.state = SYS_STATE_RUNNING;
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_MOVE_JOINT:
                if (frame.len >= 13) {  /* joint_id(1) + angle(4) + speed(4) + accel(4) */
                    cmd.msg_type = CTRL_MSG_ANGLE;
                    cmd.joint_id = frame.payload[0];
                    memcpy(&cmd.param1, &frame.payload[1], 4);
                    memcpy(&cmd.param2, &frame.payload[5], 4);
                    memcpy(&cmd.param3, &frame.payload[9], 4);
                    cmd.mode = 1;
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_MOVE_JOINTS:
                if (frame.len >= 24) {  /* angles[4](16) + speed(4) + accel(4) */
                    cmd.msg_type = CTRL_MSG_MOVE_JOINTS;
                    memcpy(&cmd.param1, &frame.payload[0],  4);
                    memcpy(&cmd.param2, &frame.payload[4],  4);
                    memcpy(&cmd.param3, &frame.payload[8],  4);
                    memcpy(&cmd.param4, &frame.payload[12], 4);
                    /* param5=speed, param6=accel */
                    memcpy(&cmd.param5, &frame.payload[16], 4);
                    memcpy(&cmd.param6, &frame.payload[20], 4);
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_MOVE_CART:
                if (frame.len >= 32) {  /* pose[6](24) + speed(4) + accel(4) */
                    cmd.msg_type = CTRL_MSG_MOVE_CARTESIAN;
                    memcpy(&cmd.param1, &frame.payload[0],  4);  /* X */
                    memcpy(&cmd.param2, &frame.payload[4],  4);  /* Y */
                    memcpy(&cmd.param3, &frame.payload[8],  4);  /* Z */
                    memcpy(&cmd.param4, &frame.payload[12], 4);  /* RX */
                    memcpy(&cmd.param5, &frame.payload[16], 4);  /* RY */
                    memcpy(&cmd.param6, &frame.payload[20], 4);  /* RZ */
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_STOP:
                cmd.msg_type = CTRL_MSG_STOP;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_GET_STATE: {
                /* 收集系统状态打包发送 */
                uint8_t state_payload[80];
                int off = 0;
                state_payload[off++] = g_sys.state;
                state_payload[off++] = g_sys.error_code;
                state_payload[off++] = g_sys.estop_triggered;
                state_payload[off++] = g_sys.motion_busy;
                for (int i = 0; i < MAX_JOINT_COUNT; i++) {
                    memcpy(&state_payload[off], &g_sys.joints[i].angle, 4);  off += 4;
                    memcpy(&state_payload[off], &g_sys.joints[i].speed, 4);  off += 4;
                    memcpy(&state_payload[off], &g_sys.joints[i].torque, 4); off += 4;
                    state_payload[off++] = g_sys.joint_online[i];
                }
                memcpy(&state_payload[off], g_sys.cart_pose, 24); off += 24;
                proto_build_response(RSP_ACK, frame.seq, state_payload, off, tx_buf, &tx_len);
                break;
            }

            case CMD_STREAM_START:
                if (frame.len >= 2) {
                    memcpy(&g_stream_rate_ms, frame.payload, 2);
                    g_stream_enabled = 1;
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_STREAM_STOP:
                g_stream_enabled = 0;
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_SET_ZERO:
                if (frame.len >= 1) {
                    cmd.msg_type = CTRL_MSG_SET_ZERO;
                    cmd.joint_id = frame.payload[0];
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_SET_PID:
                if (frame.len >= 13) {
                    /* joint_id(1) + P(4) + I(4) + D(4) */
                    uint8_t jid = frame.payload[0];
                    float P, I, D;
                    memcpy(&P, &frame.payload[1], 4);
                    memcpy(&I, &frame.payload[5], 4);
                    memcpy(&D, &frame.payload[9], 4);
                    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
                    rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
                    set_pid(jid, P, I, D);
                    rt_mutex_release(mutex_can_rx);
                    rt_mutex_release(mutex_can_tx);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_REBOOT_JOINT:
                if (frame.len >= 1) {
                    cmd.msg_type = CTRL_MSG_REBOOT;
                    cmd.joint_id = frame.payload[0];
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            /* ---- 标定命令 ---- */
            case CMD_CALIB_START:
                if (frame.len >= 1) {
                    cmd.msg_type = CTRL_MSG_CALIB_START;
                    cmd.joint_id = frame.payload[0];
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_CALIB_END:
                cmd.msg_type = CTRL_MSG_CALIB_END;
                rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;

            case CMD_SET_SYS_ORIGIN:
                if (frame.len >= 12) {
                    cmd.msg_type = CTRL_MSG_SET_SYS_ORIGIN;
                    memcpy(&cmd.param1, &frame.payload[0], 4);
                    memcpy(&cmd.param2, &frame.payload[4], 4);
                    memcpy(&cmd.param3, &frame.payload[8], 4);
                    rt_mb_send(mb_ctrl_cmd, (rt_ubase_t)&cmd);
                    proto_build_response(RSP_ACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                } else {
                    proto_build_response(RSP_INVALID_PARAM, frame.seq, NULL, 0, tx_buf, &tx_len);
                }
                break;

            case CMD_GET_SYS_ORIGIN: {
                uint8_t origin[12];
                memcpy(&origin[0], (void *)&g_sys.base_offset[0], 4);
                memcpy(&origin[4], (void *)&g_sys.base_offset[1], 4);
                memcpy(&origin[8], (void *)&g_sys.base_offset[2], 4);
                proto_build_response(RSP_ACK, frame.seq, origin, 12, tx_buf, &tx_len);
                break;
            }

            case CMD_GET_CALIB_STATUS: {
                uint8_t st[1 + 4 + 16 + 16];
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

            default:
                proto_build_response(RSP_NACK, frame.seq, NULL, 0, tx_buf, &tx_len);
                break;
            }
        }

        /* ---- 遥测发送 ---- */
        if (g_stream_enabled) {
            result = rt_mb_recv(mb_telemetry, &msg_val, 0);  /* non-blocking */
            if (result == RT_EOK) {
                memcpy(&telem, (void *)msg_val, sizeof(telem));
                /* 打包遥测帧并通过 UART 发送 */
                proto_build_telemetry(seq++, (uint8_t *)&telem, sizeof(telem),
                                      tx_buf, &tx_len);
                /* UART 发送 tx_buf, tx_len 字节... */
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

    watchdog_thread = rt_thread_create("watchdog", watchdog_thread_entry,
                                       RT_NULL, 512, 2, 5);
    if (watchdog_thread != RT_NULL) rt_thread_startup(watchdog_thread);

    joint_data_thread = rt_thread_create("joint_data", joint_data_thread_entry,
                                         RT_NULL, 2048, 5, 5);
    if (joint_data_thread != RT_NULL) rt_thread_startup(joint_data_thread);

    safety_thread = rt_thread_create("safety", safety_thread_entry,
                                     RT_NULL, 1024, 6, 5);
    if (safety_thread != RT_NULL) rt_thread_startup(safety_thread);

    joint_ctrl_thread = rt_thread_create("joint_ctrl", joint_ctrl_thread_entry,
                                         RT_NULL, 3072, 8, 5);
    if (joint_ctrl_thread != RT_NULL) rt_thread_startup(joint_ctrl_thread);

    comm_thread = rt_thread_create("comm", comm_thread_entry,
                                   RT_NULL, 1536, 15, 5);
    if (comm_thread != RT_NULL) rt_thread_startup(comm_thread);

    led_thread = rt_thread_create("led", led_thread_entry,
                                  RT_NULL, 512, 22, 5);
    if (led_thread != RT_NULL) rt_thread_startup(led_thread);

    /* 3. 标记系统就绪 */
    g_sys.state = SYS_STATE_READY;
    rt_kprintf("[Init] ===== All 6 threads started, system ready =====\n");

    return RT_EOK;
}
