/**
 ******************************************************************************
 * @file    calibration.c
 * @brief   标定模块实现 — 零点/方向/Flash 持久化 (Pi v1.2)
 * @date    2026-08-08
 ******************************************************************************
 */

#include "calibration.h"
#include "calib_defaults.h"
#include "comm_protocol.h"    /* proto_crc16 */
#include <rtthread.h>
#include <math.h>
#include <string.h>

/* STM32F4 Flash HAL */
#include "stm32f4xx_hal.h"

/* ---- 外部依赖 ---- */
extern rt_mutex_t mutex_can_tx;
extern rt_mutex_t mutex_can_rx;

/* Flash 扇区末地址 (SECTOR_11: 0x080E0000 ~ 0x080FFFFF, 128KB) */
#define FLASH_SECTOR_11_START  0x080E0000UL
#define FLASH_SECTOR_11_END    0x080FFFFFUL

/* ---- 初始化: 尝试从 Flash 加载 ---- */
int calib_init(struct calib_ctx *ctx, float drift_threshold)
{
    if (ctx == NULL) return -1;

    memset(ctx, 0, sizeof(*ctx));
    ctx->drift_threshold = (drift_threshold > 0.0f)
                           ? drift_threshold : CALIB_ZERO_DRIFT_DEG;
    ctx->calib_joint_id = 0;
    ctx->check_rounds   = 0;

    /* 设置默认方向为 +1 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        ctx->encoder_direction[i] = 1;

    /* 尝试加载 Flash 标定数据 */
    if (calib_load_from_flash(ctx) == 0) {
        ctx->state = CALIB_ZERO_OK;
        return 0;
    }

    /* Flash 无有效数据, 进入未标定状态 */
    ctx->state = CALIB_ZERO_LOST;
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        ctx->zero_valid[i] = 0;
    return -1;
}

/* ---- 启动上电零点校验 ---- */
void calib_start_check(struct calib_ctx *ctx)
{
    if (ctx == NULL) return;
    ctx->state            = CALIB_ZERO_CHECKING;
    ctx->check_start_tick = rt_tick_get();
    ctx->check_rounds     = 0;
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        ctx->zero_valid[i] = 0;
}

/* ---- 零点校验更新 ---- */
int calib_check_update(struct calib_ctx *ctx, const float joint_angles[MAX_JOINT_COUNT],
                       uint32_t now_tick)
{
    if (ctx == NULL || joint_angles == NULL) return -1;
    if (ctx->state != CALIB_ZERO_CHECKING)   return 0;

    ctx->check_rounds++;
    if (ctx->check_rounds < CALIB_CHECK_MIN_ROUNDS)
        return 0;

    int all_ok = 1;
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (fabsf(joint_angles[i]) <= ctx->drift_threshold) {
            ctx->zero_valid[i] = 1;
        } else {
            all_ok = 0;
            ctx->zero_valid[i] = 0;
        }
    }

    if (all_ok) {
        ctx->state = CALIB_ZERO_OK;
        return 1;
    }

    uint32_t elapsed = now_tick - ctx->check_start_tick;
    if (elapsed > rt_tick_from_millisecond(CALIB_CHECK_TIMEOUT_MS)) {
        ctx->state = CALIB_ZERO_LOST;
        return -1;
    }
    return 0;
}

/* ---- 进入单关节标定模式 ---- */
int calib_start_joint(struct calib_ctx *ctx, uint8_t joint_id)
{
    if (ctx == NULL) return -1;
    if (ctx->state != CALIB_ZERO_OK && ctx->state != CALIB_CALIBRATING)
        return -1;
    if (joint_id < 1 || joint_id > MAX_JOINT_COUNT) return -1;

    ctx->state          = CALIB_CALIBRATING;
    ctx->calib_joint_id = joint_id;

    /* motion_aid 助力, 方便手动拖到机械零位 */
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
    motion_aid(joint_id, 0.0f, CALIB_AID_SPEED_RPM,
               CALIB_AID_ANGLE_ERR_DEG, CALIB_AID_SPEED_ERR_RPM,
               CALIB_AID_TORQUE_NM);
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);
    return 0;
}

/* ---- 确认零点: 记录当前原始反馈作为零偏 (v1.2: 不调用 set_zero_position) ---- */
int calib_confirm_zero(struct calib_ctx *ctx, uint8_t joint_id,
                       float current_raw_angle_deg)
{
    if (ctx == NULL) return -2;
    if (ctx->state != CALIB_CALIBRATING) return -2;
    if (joint_id < 1 || joint_id > MAX_JOINT_COUNT) return -2;

    int idx = joint_id - 1;

    /* 记录原始零偏: q_model = direction * (q_raw - zero_offset) */
    ctx->zero_offset_raw_deg[idx] = current_raw_angle_deg;
    ctx->set_zero_sample_deg      = current_raw_angle_deg;
    ctx->set_zero_verified        = 0;

    /* 立即读回验证: 模型角度应 ≈ 0 */
    float model = calib_raw_to_model(ctx, joint_id, current_raw_angle_deg);
    if (fabsf(model) <= ctx->drift_threshold) {
        ctx->set_zero_verified = 1;
        ctx->zero_valid[idx]   = 1;
        return 0;
    }

    ctx->zero_valid[idx] = 0;
    return -1;
}

/* ---- 读回验证 ---- */
int calib_verify_zero(struct calib_ctx *ctx, uint8_t joint_id,
                      float current_model_angle_deg)
{
    if (ctx == NULL) return -1;
    if (joint_id < 1 || joint_id > MAX_JOINT_COUNT) return -1;

    if (fabsf(current_model_angle_deg) <= TRAJ_ARRIVAL_TOLERANCE) {
        ctx->zero_valid[joint_id - 1] = 1;
        return 0;
    }
    return -1;
}

/* ---- 退出标定模式 ---- */
void calib_end(struct calib_ctx *ctx)
{
    if (ctx == NULL) return;

    int all_ok = 1;
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (!ctx->zero_valid[i]) { all_ok = 0; break; }
    }
    ctx->state          = all_ok ? CALIB_ZERO_OK : CALIB_ZERO_LOST;
    ctx->calib_joint_id = 0;
}

/* ---- 记录空载力矩基线 ---- */
void calib_record_torque_baseline(struct calib_ctx *ctx, const float torques[MAX_JOINT_COUNT])
{
    if (ctx == NULL || torques == NULL) return;
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        ctx->torque_baseline[i] = torques[i];
}

/* ========================================================================== */
/* v1.2: 模型 ↔ 原始角度变换                                                  */
/* ========================================================================== */

float calib_raw_to_model(const struct calib_ctx *ctx, uint8_t joint_id,
                         float raw_angle_deg)
{
    if (ctx == NULL || joint_id < 1 || joint_id > MAX_JOINT_COUNT)
        return raw_angle_deg;
    int idx = joint_id - 1;
    return (float)ctx->encoder_direction[idx] *
           (raw_angle_deg - ctx->zero_offset_raw_deg[idx]);
}

float calib_model_to_raw(const struct calib_ctx *ctx, uint8_t joint_id,
                         float model_angle_deg)
{
    if (ctx == NULL || joint_id < 1 || joint_id > MAX_JOINT_COUNT)
        return model_angle_deg;
    int idx = joint_id - 1;
    return ctx->zero_offset_raw_deg[idx] +
           (float)ctx->encoder_direction[idx] * model_angle_deg;
}

void calib_raw_to_model_all(const struct calib_ctx *ctx,
                            const float raw_deg[MAX_JOINT_COUNT],
                            float model_deg_out[MAX_JOINT_COUNT])
{
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        model_deg_out[i] = calib_raw_to_model(ctx, i + 1, raw_deg[i]);
}

void calib_model_to_raw_all(const struct calib_ctx *ctx,
                            const float model_deg[MAX_JOINT_COUNT],
                            float raw_deg_out[MAX_JOINT_COUNT])
{
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        raw_deg_out[i] = calib_model_to_raw(ctx, i + 1, model_deg[i]);
}

/* ========================================================================== */
/* v1.2: Flash 持久化                                                         */
/* ========================================================================== */

/* 计算 Flash record 的 CRC-16 (覆盖 version 及之后的所有字段) */
static uint16_t calib_flash_crc(const struct calib_flash_record *rec)
{
    /* CRC over [version + crc16(0) + zero_offset[] + direction[] + reserved[]] */
    uint8_t buf[2 + 2 + MAX_JOINT_COUNT * 4 + MAX_JOINT_COUNT + 2];
    int off = 0;
    memcpy(&buf[off], &rec->version, 2); off += 2;
    /* crc16 字段填 0 */
    buf[off++] = 0; buf[off++] = 0;
    memcpy(&buf[off], rec->zero_offset_raw_deg, MAX_JOINT_COUNT * 4); off += MAX_JOINT_COUNT * 4;
    memcpy(&buf[off], rec->encoder_direction, MAX_JOINT_COUNT); off += MAX_JOINT_COUNT;
    memcpy(&buf[off], rec->reserved, 2); off += 2;
    return proto_crc16(buf, off);
}

int calib_save_to_flash(const struct calib_ctx *ctx)
{
    if (ctx == NULL) return -1;

    struct calib_flash_record rec;
    uint32_t *src;
    HAL_StatusTypeDef hal_ret;

    memset(&rec, 0xFF, sizeof(rec));  /* Flash 擦除后为 0xFF */
    rec.magic   = CALIB_FLASH_MAGIC;
    rec.version = CALIB_FLASH_VERSION;
    rec.crc16   = 0;  /* 先填 0, 最后计算 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        rec.zero_offset_raw_deg[i] = ctx->zero_offset_raw_deg[i];
        rec.encoder_direction[i]   = ctx->encoder_direction[i];
    }
    rec.reserved[0] = 0;
    rec.reserved[1] = 0;

    /* 计算 CRC */
    rec.crc16 = calib_flash_crc(&rec);

    /* 解锁 Flash */
    HAL_FLASH_Unlock();

    /* 擦除扇区 11 */
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_SECTOR_11;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;
    hal_ret = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (hal_ret != HAL_OK) {
        HAL_FLASH_Lock();
        return -2;
    }

    /* 按 32-bit 字写入 */
    src = (uint32_t *)&rec;
    uint32_t addr = FLASH_SECTOR_11_START;
    uint32_t words = (sizeof(rec) + 3) / 4;
    for (uint32_t i = 0; i < words; i++) {
        hal_ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)src[i]);
        if (hal_ret != HAL_OK) {
            HAL_FLASH_Lock();
            return -3;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return 0;
}

int calib_load_from_flash(struct calib_ctx *ctx)
{
    if (ctx == NULL) return -1;

    const struct calib_flash_record *rec =
        (const struct calib_flash_record *)FLASH_SECTOR_11_START;

    /* 检查 magic */
    if (rec->magic != CALIB_FLASH_MAGIC)
        return -2;

    /* 检查版本 */
    if (rec->version != CALIB_FLASH_VERSION)
        return -3;

    /* 校验 CRC */
    if (rec->crc16 != calib_flash_crc(rec))
        return -4;

    /* 数据有效, 加载 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        ctx->zero_offset_raw_deg[i] = rec->zero_offset_raw_deg[i];
        ctx->encoder_direction[i]   = rec->encoder_direction[i];
        /* 验证 direction 合法性 */
        if (ctx->encoder_direction[i] != 1 && ctx->encoder_direction[i] != -1)
            ctx->encoder_direction[i] = 1;
        ctx->zero_valid[i] = 1;
    }

    return 0;
}

int calib_flash_is_valid(void)
{
    const struct calib_flash_record *rec =
        (const struct calib_flash_record *)FLASH_SECTOR_11_START;
    if (rec->magic != CALIB_FLASH_MAGIC) return 0;
    if (rec->version != CALIB_FLASH_VERSION) return 0;
    if (rec->crc16 != calib_flash_crc(rec)) return 0;
    return 1;
}
