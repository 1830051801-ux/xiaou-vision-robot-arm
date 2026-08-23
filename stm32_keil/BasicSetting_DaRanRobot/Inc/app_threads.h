/**
 ******************************************************************************
 * @file    app_threads.h
 * @brief   4轴机械臂 — RT-Thread 多线程架构定义 (精简版)
 * @author  YaowenLi
 * @date    2026-07-27 (架构重构: 算法层迁移至 ROS2, MCU 退化为执行器)
 ******************************************************************************
 * @attention
 * 重构后线程架构 (6线程):
 *   P=2  → 看门狗复位       (512B)   IWDG feed + 心跳监控
 *   P=5  → 关节数据接收     (2048B)  CAN 总线数据实时解析 (4关节)
 *   P=6  → 保护与限位       (1024B)  安全检测与急停
 *   P=8  → 关节控制         (2048B)  轨迹点执行 + 零点校验 + 标定 (精简)
 *   P=15 → 通讯（串口）     (1536B)  协议解析 + 命令分发 + 遥测
 *   P=22 → LED 指示灯       (512B)   系统状态指示
 *
 * 重构后数据流:
 *   CAN_IRQ → joint_data → mb_joint_data → safety (安全检查)
 *                                  |
 *   UART_RX → comm → mb_ctrl_cmd → joint_ctrl → CAN_TX (轨迹点执行)
 *                       mb_telemetry ← joint_ctrl (关节状态回传)
 *
 * 算法层 (IK/轨迹规划/抓取序列/重力补偿) 已迁移至树莓派 ROS2。
 ******************************************************************************
 */

#ifndef __APP_THREADS_H__
#define __APP_THREADS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "DrEmpower_can.h"
#include "arm_config.h"

/* 系统配置常量 ---------------------------------------------------------------*/
/* MAX_JOINT_COUNT 定义在 arm_config.h */
#define MAILBOX_MAX_MSG     32      /* 邮箱消息队列深度 (6关节×5周期缓冲) */
#define MAX_TRAJ_WAYPOINTS  32      /* 轨迹点缓冲区最大容量 */

/* -------------------------------------------------------------------------- */
/* 系统运行状态枚举                                                           */
/* -------------------------------------------------------------------------- */
enum sys_state {
    SYS_STATE_INIT          = 0x00, /* 系统初始化中 */
    SYS_STATE_READY         = 0x01, /* 系统就绪 */
    SYS_STATE_RUNNING       = 0x02, /* 系统运行中 */
    SYS_STATE_ESTOP         = 0x03, /* 急停状态 */
    SYS_STATE_ERROR         = 0x04, /* 故障状态 */
    SYS_STATE_ZERO_CHECK    = 0x06, /* 上电零点校验中 */
    SYS_STATE_CALIBRATION   = 0x07, /* 标定模式 */
};

/* 错误码枚举 */
enum error_code {
    ERR_NONE                = 0x00, /* 无错误 */
    ERR_CAN_TIMEOUT         = 0x01, /* CAN 通信超时 */
    ERR_JOINT_OVERCURRENT   = 0x02, /* 关节过流 */
    ERR_JOINT_OVERTEMP      = 0x03, /* 关节过热 */
    ERR_JOINT_COMM_LOSS     = 0x04, /* 关节通信丢失 */
    ERR_LIMIT_TRIGGERED     = 0x06, /* 限位触发 */
    ERR_TRAJ_ERROR          = 0x08, /* 轨迹执行错误 */
    ERR_HOST_COMM_LOSS      = 0x09, /* 上位机通信丢失 */
    ERR_ZERO_LOST           = 0x0A, /* 关节零点丢失 */
    ERR_NOT_CALIBRATED      = 0x0B, /* 零点未标定 */
};

/* -------------------------------------------------------------------------- */
/* 线程间通信 — 消息结构                                                      */
/* -------------------------------------------------------------------------- */

/* 控制指令消息类型 (精简: 移除笛卡尔/示教, 新增轨迹点/夹爪) */
enum ctrl_msg_type {
    /* 轨迹执行 */
    CTRL_MSG_TRAJ_POINT     = 0x50, /* 轨迹点执行 */
    CTRL_MSG_TRAJ_CLEAR     = 0x51, /* 清空轨迹缓冲区 */
    CTRL_MSG_STOP           = 0x12, /* 停止运动 */

    /* 单关节控制 (保留, 调试用) */
    CTRL_MSG_ANGLE          = 0x01, /* 单关节角度控制 */
    CTRL_MSG_SPEED          = 0x02, /* 单关节转速控制 */
    CTRL_MSG_TORQUE         = 0x03, /* 单关节力矩控制 */

    /* 系统控制 */
    CTRL_MSG_ESTOP          = 0x04, /* 急停 */
    CTRL_MSG_CLEAR_ERROR    = 0x05, /* 清除错误 */

    /* 标定 */
    CTRL_MSG_SET_ZERO       = 0x06, /* 设置零点 */
    CTRL_MSG_CALIB_START    = 0x14, /* 开始单关节标定 */
    CTRL_MSG_CALIB_END      = 0x15, /* 结束标定 */

    /* 夹爪 */
    CTRL_MSG_GRIPPER        = 0x52, /* 夹爪角度控制 */

    /* 参数设置 */
    CTRL_MSG_SET_PID        = 0x31, /* 设置 PID */
    CTRL_MSG_SET_MODE       = 0x32, /* 设置模式 */
    CTRL_MSG_REBOOT         = 0x08, /* 关节重启 */
    CTRL_MSG_QUERY_STATE    = 0x09, /* 查询状态 */
    CTRL_MSG_SAVE_CONFIG    = 0x34, /* v1.2: 保存标定到 Flash */
};

/* 控制指令消息体（通过邮箱传递） */
struct ctrl_msg {
    rt_uint8_t  msg_type;                   /* 消息类型 (enum ctrl_msg_type) */
    rt_uint8_t  joint_id;                   /* 目标关节 ID */
    rt_uint8_t  joint_count;                /* 多关节控制时的关节数量 (轨迹: 固定6) */
    rt_uint8_t  mode;                       /* 控制模式 / 轨迹时用作批量帧数 */
    float       param1;                     /* 参数1：角度 / 转速 / 力矩 / J1 */
    float       param2;                     /* 参数2：转速 / 加转速 / 前馈 / J2 */
    float       param3;                     /* 参数3：滤波带宽 / 力矩 / J3 */
    float       param4;                     /* 参数4：J4 / 备用 */
    float       param5;                     /* 参数5：J5 角度 / 备用 */
    float       param6;                     /* 参数6：J6 角度 / 备用 */
    float       param7;                     /* 参数7：traj_duration_ms (轨迹用时) */
};

/* 关节数据消息（CAN 接收线程发布） */
struct joint_data_msg {
    rt_uint8_t  joint_id;                   /* 关节 ID (1~4) */
    float       angle;                      /* 角度 (°) */
    float       speed;                      /* 转速 (r/min) */
    float       torque;                     /* 力矩 (Nm) */
    float       voltage;                    /* 电压 (V) */
    float       current;                    /* 电流 (A) */
    rt_uint32_t timestamp;                  /* 时间戳 (ms) */
};

/* 遥测响应消息（joint_ctrl → comm_thread → UART 上传） */
struct telemetry_msg {
    rt_uint32_t timestamp;                  /* 系统时间戳 (ms) */
    struct joint_data_msg joints[MAX_JOINT_COUNT];        /* 4关节完整状态 */
    rt_uint8_t  sys_state;                  /* 系统状态 */
    rt_uint8_t  error_code;                 /* 错误码 */
    rt_uint8_t  motion_busy;                /* 运动中标志 */
    rt_uint8_t  estop_triggered;            /* 急停标志 */
};

/* -------------------------------------------------------------------------- */
/* 系统全局状态（多线程共享，互斥保护）                                        */
/* -------------------------------------------------------------------------- */
struct sys_status {
    volatile rt_uint8_t state;              /* 系统状态 enum sys_state */
    volatile rt_uint8_t error_code;         /* 错误码 enum error_code */
    volatile rt_uint8_t estop_triggered;    /* 急停触发标志 */
    volatile rt_uint32_t system_tick;       /* 系统运行 tick 计数 */

    /* 关节实时状态快照（joint_data 线程更新） */
    struct joint_data_msg  joints[MAX_JOINT_COUNT];
    volatile rt_uint8_t    joint_online[MAX_JOINT_COUNT]; /* 关节在线标志 */

    /* 运动状态 */
    volatile rt_uint8_t    motion_busy;     /* 运动执行中标志 */

    /* 标定 (calibration 模块管理) */
    volatile rt_uint8_t    zero_valid[MAX_JOINT_COUNT];   /* 各关节零点有效标志 */
    volatile float         torque_baseline[MAX_JOINT_COUNT]; /* 空载力矩基线 (Nm) */

    /* 各线程心跳计数器 (watchdog 监控) — 6线程 */
    volatile rt_uint32_t thread_heartbeat[6];

    /* 系统监控 (sysmonitor) */
    volatile rt_uint16_t modules_ready;      /* 模块就绪位图 */
    volatile rt_uint32_t uptime_ms;          /* 运行时长 */
    volatile rt_uint32_t cmd_count;          /* 命令计数 */
};

/* -------------------------------------------------------------------------- */
/* 全局 IPC 对象 extern 声明                                                   */
/* -------------------------------------------------------------------------- */

/* 线程句柄 (6 线程) */
extern rt_thread_t led_thread;
extern rt_thread_t joint_data_thread;
extern rt_thread_t joint_ctrl_thread;
extern rt_thread_t safety_thread;
extern rt_thread_t comm_thread;
extern rt_thread_t watchdog_thread;

/* 系统全局状态 */
extern struct sys_status g_sys;
/* v1.4: 安全线程 CAN 时间戳, clear 时复位 */
extern rt_tick_t g_safety_can_tick[MAX_JOINT_COUNT];

/* v2: 轨迹 Quintic 插值状态 (forward decl, 完整定义在 trajectory.h) */
struct traj_interp_state;
extern struct traj_interp_state g_interp;

/* 标定上下文 (v2: trajectory 模块需要) */
extern struct calib_ctx g_calib;

/* 关节限位常量 (v2: trajectory 模块需要) */
extern const float g_joint_angle_min[MAX_JOINT_COUNT];
extern const float g_joint_angle_max[MAX_JOINT_COUNT];
extern const float g_joint_speed_max[MAX_JOINT_COUNT];

/* 信号量 */
extern rt_sem_t sem_joint_data_ready;
extern rt_sem_t sem_uart_rx_done;
extern rt_sem_t sem_can_rx_done;

/* 互斥量 */
extern rt_mutex_t mutex_sys_state;
extern rt_mutex_t mutex_can_tx;
extern rt_mutex_t mutex_can_rx;

/* 邮箱 */
extern rt_mailbox_t mb_ctrl_cmd;
extern rt_mailbox_t mb_joint_data;
extern rt_mailbox_t mb_telemetry;

/* 遥测流控制 (调试命令行可读写) */
extern volatile int g_stream_enabled;
extern volatile int g_stream_rate_ms;

/* -------------------------------------------------------------------------- */
/* 线程入口函数声明 (6 线程)                                                  */
/* -------------------------------------------------------------------------- */

void led_thread_entry(void *parameter);
void joint_data_thread_entry(void *parameter);
void joint_ctrl_thread_entry(void *parameter);
void safety_thread_entry(void *parameter);
void comm_thread_entry(void *parameter);
void watchdog_thread_entry(void *parameter);

int app_threads_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_THREADS_H__ */
