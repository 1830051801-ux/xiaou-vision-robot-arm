/**
 ******************************************************************************
 * @file    calibration.h
 * @brief   标定模块 — 零点/方向/Flash 持久化 (Pi v1.2 架构决策 A/B)
 * @date    2026-08-08
 ******************************************************************************
 * @attention
 * 决策 A: encoder_direction / zero_offset 由 MCU 保存、加载和应用
 *          Pi 只发送模型角度；MCU 负责模型↔原始的双向变换
 *
 * 决策 B: 本模块独立于 UART 线程；Flash 写入在 joint_ctrl 线程中完成
 *
 * 变换公式 (逐轴):
 *   q_model_feedback = encoder_direction * (q_raw_feedback - zero_offset_raw)
 *   q_raw_command    = zero_offset_raw + encoder_direction * q_model_target
 *
 * encoder_direction[i] 只能为 +1 或 -1
 ******************************************************************************
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arm_config.h"

/* -------------------------------------------------------------------------- */
/* Flash 持久化数据结构 (写入 STM32F407 内部 Flash)                            */
/* -------------------------------------------------------------------------- */

#define CALIB_FLASH_MAGIC        0x4441524EUL   /* "DARN" (DaRan) */
#define CALIB_FLASH_VERSION      1               /* 结构版本 */
#define CALIB_FLASH_SECTOR       FLASH_SECTOR_11 /* 末扇区, 不干扰固件 */

/* Flash 中存储的标定数据 (32 字节对齐便于 CRC) */
struct calib_flash_record {
    uint32_t magic;                             /* 0x4441524E */
    uint16_t version;                           /* 结构版本 */
    uint16_t crc16;                             /* CRC-16/MODBUS over 后续字段 */
    float    zero_offset_raw_deg[MAX_JOINT_COUNT]; /* 各轴原始零偏 (°) */
    int8_t   encoder_direction[MAX_JOINT_COUNT];   /* +1 或 -1 */
    uint8_t  reserved[2];                       /* 对齐填充 */
};

/* -------------------------------------------------------------------------- */
/* 标定状态枚举                                                               */
/* -------------------------------------------------------------------------- */
enum calib_state {
    CALIB_ZERO_UNCHECKED = 0,   /* 尚未校验/Flash 未加载 */
    CALIB_ZERO_CHECKING  = 1,   /* 校验进行中 */
    CALIB_ZERO_OK        = 2,   /* 零点正常 */
    CALIB_ZERO_LOST      = 3,   /* 零点丢失 (Flash CRC 错/未写入) */
    CALIB_CALIBRATING    = 4,   /* 标定模式 */
};

/* -------------------------------------------------------------------------- */
/* 标定运行时上下文                                                           */
/* -------------------------------------------------------------------------- */
struct calib_ctx {
    enum calib_state state;                     /* 当前状态 */
    uint8_t  calib_joint_id;                    /* 当前标定关节 ID (1~6, 0=无) */
    float    drift_threshold;                   /* 单关节漂移判定阈值 (°) */
    float    torque_baseline[MAX_JOINT_COUNT];  /* 空载力矩基线 (Nm) */
    uint32_t check_start_tick;                  /* 零点校验开始时刻 */
    uint8_t  zero_valid[MAX_JOINT_COUNT];       /* 各关节零点有效标志 */
    uint8_t  check_rounds;                      /* 数据轮次计数 */

    /* ---- v1.2: 方向与零偏 (MCU 负责) ---- */
    float    zero_offset_raw_deg[MAX_JOINT_COUNT];  /* 原始零偏 */
    int8_t   encoder_direction[MAX_JOINT_COUNT];     /* +1 或 -1 */

    /* ---- v1.2: 零位写入验证 ---- */
    float    set_zero_sample_deg;               /* 设零时的采样角度 */
    uint8_t  set_zero_verified;                 /* 读回验证通过标志 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/* 初始化 (加载 Flash 或标记为未标定) */
int  calib_init(struct calib_ctx *ctx, float drift_threshold);

/* ---- 零点校验 ---- */
void calib_start_check(struct calib_ctx *ctx);
int  calib_check_update(struct calib_ctx *ctx, const float joint_angles[MAX_JOINT_COUNT],
                        uint32_t now_tick);

/* ---- 单关节标定 ---- */
int  calib_start_joint(struct calib_ctx *ctx, uint8_t joint_id);

/* 确认零点: 记录当前原始反馈为零偏, encoder_direction 使用默认 +1 */
int  calib_confirm_zero(struct calib_ctx *ctx, uint8_t joint_id,
                        float current_raw_angle_deg);

/* 读回验证: 设零后读取当前模型角度, 误差 <0.5° 视为通过 */
int  calib_verify_zero(struct calib_ctx *ctx, uint8_t joint_id,
                       float current_model_angle_deg);

void calib_end(struct calib_ctx *ctx);
void calib_record_torque_baseline(struct calib_ctx *ctx, const float torques[MAX_JOINT_COUNT]);

/* ---- v1.2: 模型↔原始角度变换 ---- */

/* 原始 CAN 反馈 → 模型角度 (joint_data 线程每周期调用) */
float calib_raw_to_model(const struct calib_ctx *ctx, uint8_t joint_id,
                         float raw_angle_deg);

/* 模型目标角度 → 原始 CAN 命令 (joint_ctrl 线程发 CAN 前调用) */
float calib_model_to_raw(const struct calib_ctx *ctx, uint8_t joint_id,
                         float model_angle_deg);

/* 批量变换 (6 轴) */
void calib_raw_to_model_all(const struct calib_ctx *ctx,
                            const float raw_deg[MAX_JOINT_COUNT],
                            float model_deg_out[MAX_JOINT_COUNT]);
void calib_model_to_raw_all(const struct calib_ctx *ctx,
                            const float model_deg[MAX_JOINT_COUNT],
                            float raw_deg_out[MAX_JOINT_COUNT]);

/* ---- v1.2: Flash 持久化 ---- */

/* 保存当前标定数据到 Flash. 返回 0=成功, 非0=失败 */
int  calib_save_to_flash(const struct calib_ctx *ctx);

/* 从 Flash 加载标定数据. 返回 0=成功, 非0=失败 (CRC/版本/magic 异常) */
int  calib_load_from_flash(struct calib_ctx *ctx);

/* 检查 Flash 中是否有有效标定数据 */
int  calib_flash_is_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H__ */
