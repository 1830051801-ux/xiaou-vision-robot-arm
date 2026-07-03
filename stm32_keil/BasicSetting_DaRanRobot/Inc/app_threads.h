/**
 ******************************************************************************
 * @file    app_threads.h
 * @brief   4轴机械臂 — RT-Thread 多线程架构定义
 * @author  YaowenLi
 * @date    2026-06-18 (重构: 7线程→6线程, MAX_JOINT=4)
 ******************************************************************************
 * @attention
 * 本文件定义4轴机械臂多线程架构所需的：
 *   1. 系统状态共享结构
 *   2. 线程间通信邮箱消息类型（含笛卡尔控制 / 遥测）
 *   3. 各线程入口函数声明
 *   4. 全局 IPC 句柄 extern 声明
 *
 * 线程优先级与栈大小设计：
 *   P=2  → 看门狗复位       (512B)  最高优先级，系统安全
 *   P=5  → 关节数据接收     (2048B) CAN 总线数据实时解析 (4关节)
 *   P=6  → 保护与限位       (1024B) 安全检测与急停
 *   P=8  → 关节控制         (3072B) 实时运动控制 + 轨迹插补 + 运动学
 *   P=15 → 通讯（串口）     (1536B) 上位机指令交互
 *   P=22 → LED 指示灯       (512B)  系统状态指示（最低优先级）
 *
 * 已移除: attitude_thread (原 P=10) — 固定基座无需 IMU 姿态感知
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
#define MAX_JOINT_COUNT     4       /* 4轴机械臂 */
#define MAILBOX_MAX_MSG     8       /* 邮箱消息队列深度 */
#define MAX_TRAJ_WAYPOINTS  32      /* 示教模式最大路径点数 */

/* -------------------------------------------------------------------------- */
/* 系统运行状态枚举                                                           */
/* -------------------------------------------------------------------------- */
enum sys_state {
    SYS_STATE_INIT          = 0x00, /* 系统初始化中 */
    SYS_STATE_READY         = 0x01, /* 系统就绪 */
    SYS_STATE_RUNNING       = 0x02, /* 系统运行中 */
    SYS_STATE_ESTOP         = 0x03, /* 急停状态 */
    SYS_STATE_ERROR         = 0x04, /* 故障状态 */
    SYS_STATE_TEACH         = 0x05, /* 示教模式 */
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
    ERR_ATTITUDE_SENSOR     = 0x05, /* 姿态传感器故障 (保留, 如未来加回 IMU) */
    ERR_LIMIT_TRIGGERED     = 0x06, /* 限位触发 */
    ERR_WORKSPACE_VIOLATION = 0x07, /* 工作空间超出 */
    ERR_TRAJ_ERROR          = 0x08, /* 轨迹规划错误 */
    ERR_HOST_COMM_LOSS      = 0x09, /* 上位机通信丢失 */
    ERR_ZERO_LOST           = 0x0A, /* 关节零点丢失 */
    ERR_NOT_CALIBRATED      = 0x0B, /* 零点未标定 */
};

/* -------------------------------------------------------------------------- */
/* 线程间通信 — 消息结构                                                      */
/* -------------------------------------------------------------------------- */

/* 控制指令消息类型 */
enum ctrl_msg_type {
    /* 单关节控制 */
    CTRL_MSG_ANGLE          = 0x01, /* 单关节角度控制 */
    CTRL_MSG_SPEED          = 0x02, /* 单关节转速控制 */
    CTRL_MSG_TORQUE         = 0x03, /* 单关节力矩控制 */
    CTRL_MSG_ESTOP          = 0x04, /* 急停 */
    CTRL_MSG_SET_ZERO       = 0x05, /* 设置零点 */
    CTRL_MSG_SET_MODE       = 0x06, /* 设置模式 */
    CTRL_MSG_IMPEDANCE      = 0x07, /* 阻抗控制 */
    CTRL_MSG_REBOOT         = 0x08, /* 关节重启 */
    CTRL_MSG_QUERY_STATE    = 0x09, /* 查询状态 */

    /* 多关节控制（4轴机械臂新增） */
    CTRL_MSG_MOVE_JOINTS     = 0x10, /* 多关节 PTP (关节空间) */
    CTRL_MSG_MOVE_CARTESIAN  = 0x11, /* 笛卡尔直线运动 */
    CTRL_MSG_STOP            = 0x12, /* 停止运动 */
    CTRL_MSG_TEACH_MODE      = 0x13, /* 进入/退出示教模式 */
    CTRL_MSG_CALIB_START     = 0x14, /* 开始单关节标定 */
    CTRL_MSG_CALIB_END       = 0x15, /* 结束标定 */
    CTRL_MSG_SET_SYS_ORIGIN  = 0x16, /* 设置系统原点偏移 */
};

/* 控制指令消息体（通过邮箱传递） */
struct ctrl_msg {
    rt_uint8_t  msg_type;                   /* 消息类型 (enum ctrl_msg_type) */
    rt_uint8_t  joint_id;                   /* 目标关节 ID */
    rt_uint8_t  joint_count;                /* 多关节控制时的关节数量 */
    rt_uint8_t  mode;                       /* 控制模式 */
    float       param1;                     /* 参数1：角度 / 转速 / 力矩 / X */
    float       param2;                     /* 参数2：转速 / 加转速 / 前馈 / Y */
    float       param3;                     /* 参数3：滤波带宽 / 力矩 / Z */
    float       param4;                     /* 参数4：RX / 备用 */
    float       param5;                     /* 参数5：RY / 备用 (笛卡尔扩展) */
    float       param6;                     /* 参数6：RZ / 备用 (笛卡尔扩展) */
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

/* 遥测响应消息（joint_ctrl / safety → comm_thread → UART 上传） */
struct telemetry_msg {
    rt_uint32_t timestamp;                  /* 系统时间戳 (ms) */
    struct joint_data_msg joints[4];        /* 4关节完整状态 */
    rt_uint8_t  sys_state;                  /* 系统状态 */
    rt_uint8_t  error_code;                 /* 错误码 */
    rt_uint8_t  motion_busy;                /* 运动中标志 */
    float       cart_pose[6];               /* FK 计算的末端位姿 {x,y,z,rx,ry,rz} */
};

/* -------------------------------------------------------------------------- */
/* 系统全局状态（多线程共享，互斥保护）                                       */
/* -------------------------------------------------------------------------- */
struct sys_status {
    volatile rt_uint8_t state;              /* 系统状态 enum sys_state */
    volatile rt_uint8_t error_code;         /* 错误码 enum error_code */
    volatile rt_uint8_t estop_triggered;    /* 急停触发标志 */
    volatile rt_uint32_t system_tick;       /* 系统运行 tick 计数 */

    /* 关节实时状态快照（joint_data 线程更新） */
    struct joint_data_msg  joints[MAX_JOINT_COUNT];
    volatile rt_uint8_t    joint_online[MAX_JOINT_COUNT]; /* 关节在线标志 */

    /* 笛卡尔末端位姿 (joint_ctrl 线程 FK 更新) */
    volatile float         cart_pose[6];    /* {x,y,z,rx,ry,rz} */
    volatile rt_uint8_t    motion_busy;     /* 运动执行中标志 */

    /* 标定/坐标系 (calibration 模块管理) */
    volatile float         base_offset[3];  /* 系统原点偏移 {x,y,z} (mm) */
    volatile rt_uint8_t    zero_valid[4];   /* 各关节零点有效标志 */
    volatile float         torque_baseline[4]; /* 空载力矩基线 (Nm) */

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

/* 信号量 */
extern rt_sem_t sem_joint_data_ready;       /* 关节数据就绪信号 */
extern rt_sem_t sem_uart_rx_done;           /* 串口接收完成信号 */
extern rt_sem_t sem_can_rx_done;            /* CAN RX 完成信号 (ISR→线程) */

/* 互斥量 */
extern rt_mutex_t mutex_sys_state;          /* 系统状态互斥锁 */
extern rt_mutex_t mutex_can_tx;             /* CAN TX 互斥锁 */
extern rt_mutex_t mutex_can_rx;             /* CAN RX 互斥锁 */

/* 邮箱 */
extern rt_mailbox_t mb_ctrl_cmd;            /* 控制指令邮箱 (comm→joint_ctrl) */
extern rt_mailbox_t mb_joint_data;          /* 关节数据邮箱 (joint_data→safety) */
extern rt_mailbox_t mb_telemetry;           /* 遥测数据邮箱 (joint_ctrl→comm) */

/* 已移除的 IPC (attitude 线程):
 *   sem_attitude_ready, mb_attitude, attitude_thread
 */

/* -------------------------------------------------------------------------- */
/* 线程入口函数声明 (6 线程)                                                  */
/* -------------------------------------------------------------------------- */

void led_thread_entry(void *parameter);
void joint_data_thread_entry(void *parameter);
void joint_ctrl_thread_entry(void *parameter);
void safety_thread_entry(void *parameter);
void comm_thread_entry(void *parameter);
void watchdog_thread_entry(void *parameter);

/* 线程初始化函数（在 main 中调用） */
int app_threads_init(void);

/* -------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __APP_THREADS_H__ */
