/**
 ******************************************************************************
 * @file    trajectory.c
 * @brief   轨迹模块实现 — FIFO / 校验 / 原子入队 / 到位检测
 * @date    2026-08-08
 ******************************************************************************
 */

#include "trajectory.h"
#include "app_threads.h"
#include "calibration.h"
#include "DrEmpower_can.h"
#include <rtthread.h>
#include <string.h>
#include <math.h>

/* ---- 关节角度限位 (有效限位 = min(物理, 操作), 来自 arm_config.h) ---- */
/* safety.c 需要通过 extern 引用, 因此不能是 static */
const float traj_effective_min[MAX_JOINT_COUNT] = JOINT_EFFECTIVE_MIN_LIST;
const float traj_effective_max[MAX_JOINT_COUNT] = JOINT_EFFECTIVE_MAX_LIST;

/* ---- 16 槽环形 FIFO ---- */
static float    fifo_angles[TRAJ_FIFO_CAPACITY][MAX_JOINT_COUNT];
static uint16_t fifo_duration[TRAJ_FIFO_CAPACITY];
static uint8_t  fifo_head  = 0;
static uint8_t  fifo_tail  = 0;
static uint8_t  fifo_count = 0;

/* FIFO 互斥量 */
static rt_mutex_t fifo_mutex = RT_NULL;

/* ---- 初始化 ---- */
void traj_fifo_init(void)
{
    if (fifo_mutex == RT_NULL)
        fifo_mutex = rt_mutex_create("traj_fifo", RT_IPC_FLAG_FIFO);
    fifo_head  = 0;
    fifo_tail  = 0;
    fifo_count = 0;
}

/* ---- 清空 ---- */
void traj_fifo_clear(void)
{
    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);
    fifo_head  = 0;
    fifo_tail  = 0;
    fifo_count = 0;
    if (fifo_mutex) rt_mutex_release(fifo_mutex);
}

/* ---- 剩余槽位 (v2: 互斥保护) ---- */
uint8_t traj_fifo_free(void)
{
    uint8_t free_slots;
    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);
    free_slots = TRAJ_FIFO_CAPACITY - fifo_count;
    if (fifo_mutex) rt_mutex_release(fifo_mutex);
    return free_slots;
}

/* ---- 已用槽位 (v2: 互斥保护) ---- */
uint8_t traj_fifo_count(void)
{
    uint8_t count;
    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);
    count = fifo_count;
    if (fifo_mutex) rt_mutex_release(fifo_mutex);
    return count;
}

/* ---- float 有限性检查 ---- */
int traj_is_finite(float v)
{
    uint32_t bits;
    memcpy(&bits, &v, 4);
    return (bits & 0x7F800000) != 0x7F800000;
}

/* ---- 纯校验: 26B payload -> traj_err ---- */
int traj_validate(const uint8_t payload[26])
{
    float angles[6];
    uint16_t dur;

    for (int i = 0; i < 6; i++) {
        memcpy(&angles[i], &payload[i * 4], 4);
        if (!traj_is_finite(angles[i]))
            return TRAJ_ERR_NAN_INF;
        if (angles[i] < traj_effective_min[i] ||
            angles[i] > traj_effective_max[i])
            return TRAJ_ERR_LIMIT;
    }
    memcpy(&dur, &payload[24], 2);
    if (dur < TRAJ_DUR_MIN_MS || dur > TRAJ_DUR_MAX_MS)
        return TRAJ_ERR_DURATION;

    return TRAJ_OK;
}

/* ---- 单点入队 (v2: 激活 check_estop + 状态感知校验) ---- */
int traj_enqueue_single(const uint8_t payload[26], int check_estop)
{
    int err;

    /* 纯数据校验 */
    err = traj_validate(payload);
    if (err != TRAJ_OK) return err;

    /* 状态感知校验 (v2: 由 check_estop 参数控制) */
    if (check_estop) {
        if (g_sys.estop_triggered)
            return TRAJ_ERR_ESTOP;
        if (g_sys.state == SYS_STATE_INIT ||
            g_sys.state == SYS_STATE_ZERO_CHECK ||
            g_sys.state == SYS_STATE_CALIBRATION)
            return TRAJ_ERR_BUSY;
        /* 检查在线关节的零点有效性 */
        int any_online = 0;
        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            if (g_sys.joint_online[i]) {
                any_online = 1;
                if (!g_sys.zero_valid[i])
                    return TRAJ_ERR_BUSY;
            }
        }
        if (!any_online) return TRAJ_ERR_BUSY;
    }

    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);

    if (fifo_count >= TRAJ_FIFO_CAPACITY) {
        if (fifo_mutex) rt_mutex_release(fifo_mutex);
        return TRAJ_ERR_FIFO_FULL;
    }

    /* 写入 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        memcpy(&fifo_angles[fifo_head][i], &payload[i * 4], 4);
    uint16_t dur;
    memcpy(&dur, &payload[24], 2);
    fifo_duration[fifo_head] = dur;
    fifo_head = (fifo_head + 1) % TRAJ_FIFO_CAPACITY;
    fifo_count++;

    if (fifo_mutex) rt_mutex_release(fifo_mutex);
    return TRAJ_OK;
}

/* ---- 批量原子入队 (v2: 激活 check_estop + 状态感知校验) ---- */
int traj_enqueue_batch(const uint8_t *payload, uint8_t n, int check_estop)
{
    int err;

    if (n < 1 || n > TRAJ_BATCH_MAX)
        return TRAJ_ERR_LEN;

    /* 状态感知校验 (v2: 一次性检查, 对批量所有点有效) */
    if (check_estop) {
        if (g_sys.estop_triggered)
            return TRAJ_ERR_ESTOP;
        if (g_sys.state == SYS_STATE_INIT ||
            g_sys.state == SYS_STATE_ZERO_CHECK ||
            g_sys.state == SYS_STATE_CALIBRATION)
            return TRAJ_ERR_BUSY;
        int any_online = 0;
        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            if (g_sys.joint_online[i]) {
                any_online = 1;
                if (!g_sys.zero_valid[i])
                    return TRAJ_ERR_BUSY;
            }
        }
        if (!any_online) return TRAJ_ERR_BUSY;
    }

    /* 第一遍: 校验全部 */
    for (uint8_t k = 0; k < n; k++) {
        err = traj_validate(&payload[k * 26]);
        if (err != TRAJ_OK) return err;
    }

    /* 第二遍: 原子写入 */
    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);

    if ((TRAJ_FIFO_CAPACITY - fifo_count) < n) {
        if (fifo_mutex) rt_mutex_release(fifo_mutex);
        return TRAJ_ERR_FIFO_FULL;
    }

    for (uint8_t k = 0; k < n; k++) {
        uint16_t off = k * 26;
        for (int i = 0; i < MAX_JOINT_COUNT; i++)
            memcpy(&fifo_angles[fifo_head][i], &payload[off + i * 4], 4);
        uint16_t dur;
        memcpy(&dur, &payload[off + 24], 2);
        fifo_duration[fifo_head] = dur;
        fifo_head = (fifo_head + 1) % TRAJ_FIFO_CAPACITY;
        fifo_count++;
    }

    if (fifo_mutex) rt_mutex_release(fifo_mutex);
    return (int)n;  /* 返回入队数量 */
}

/* ---- 出队 (joint_ctrl 调用) ---- */
int traj_dequeue(float angles_out[MAX_JOINT_COUNT], uint16_t *duration_ms_out)
{
    if (fifo_mutex) rt_mutex_take(fifo_mutex, RT_WAITING_FOREVER);

    if (fifo_count == 0) {
        if (fifo_mutex) rt_mutex_release(fifo_mutex);
        return -1;
    }

    for (int i = 0; i < MAX_JOINT_COUNT; i++)
        angles_out[i] = fifo_angles[fifo_tail][i];
    *duration_ms_out = fifo_duration[fifo_tail];
    fifo_tail = (fifo_tail + 1) % TRAJ_FIFO_CAPACITY;
    fifo_count--;

    if (fifo_mutex) rt_mutex_release(fifo_mutex);
    return 0;
}

/* ---- 轨迹点启动 (v2: 每关节独立转速 → CAN) ---- */
/*
 * 速度公式: speed_rpm[i] = |q_target[i] - q_current[i]| / (duration_ms/1000) / 6
 *
 * 安全策略:
 *   - delta < 0.02° → 跳过 (无需运动)
 *   - speed < 0.1 r/min → 视为静止, 跳过
 *   - speed < JOINT_MIN_CONTINUOUS_RPM → mode=0 (轨迹跟踪, 低速插值)
 *   - speed >= JOINT_MIN_CONTINUOUS_RPM → mode=1 (梯形速度剖面)
 *   - speed > g_joint_speed_max[i] → 钳位到最大转速 + 警告
 *   - 全部跳过 → 返回 0 (无关节需要运动)
 *
 * 注意: 使用 set_angle() 逐关节发送 (非 set_angles() 广播触发),
 *       因为每关节转速不同, 不能共用统一的运动时间 t。
 *       CAN 总线延迟约 1ms/帧, 对机械运动影响可忽略。
 */
int traj_start_point(const float target_model_deg[MAX_JOINT_COUNT],
                     uint16_t duration_ms,
                     const uint8_t joint_online[MAX_JOINT_COUNT])
{
    float q_current_model[MAX_JOINT_COUNT];
    float raw_target[MAX_JOINT_COUNT];
    float speed_rpm[MAX_JOINT_COUNT];
    uint8_t do_move[MAX_JOINT_COUNT];
    uint8_t any_moving = 0;
    float dur_sec;

    if (duration_ms == 0) return TRAJ_ERR_DURATION;
    dur_sec = duration_ms / 1000.0f;

    /* 第一遍: 计算每关节转速 + 模型→原始转换 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        do_move[i] = 0;
        speed_rpm[i] = 0.0f;

        if (!joint_online[i]) continue;

        q_current_model[i] = g_sys.joints[i].angle;
        raw_target[i] = calib_model_to_raw(&g_calib, i + 1, target_model_deg[i]);

        float raw_current = calib_model_to_raw(&g_calib, i + 1, q_current_model[i]);
        float delta = fabsf(raw_target[i] - raw_current);

        if (delta < 0.02f) continue;  /* 无需运动 */

        float spd = delta / dur_sec / 6.0f;  /* speed_rpm = deg / sec / 6 */
        if (spd < 0.1f) continue;            /* 极小速度, 视为静止 */

        /* 限速检查 */
        if (spd > g_joint_speed_max[i]) {
            rt_kprintf("[Traj] WARN: J%d speed %.1f > max %.1f, clamping\n",
                       i + 1, spd, g_joint_speed_max[i]);
            spd = g_joint_speed_max[i];
        }

        speed_rpm[i]  = spd;
        do_move[i]    = 1;
        any_moving    = 1;
    }

    if (!any_moving) return TRAJ_OK;  /* 所有关节已到位 */

    /* 第二遍: 发送 CAN 指令 (逐关节) */
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);

    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (!do_move[i]) continue;

        if (speed_rpm[i] >= JOINT_MIN_CONTINUOUS_RPM) {
            /* mode=1 梯形速度剖面 */
            set_angle((uint8_t)(CAN_ID_JOINT_1 + i), raw_target[i],
                      speed_rpm[i], 100.0f, 1);
        } else {
            /* mode=0 轨迹跟踪 (低于最小连续转速, 等待 Commit B 10ms 插值) */
            set_angle((uint8_t)(CAN_ID_JOINT_1 + i), raw_target[i],
                      speed_rpm[i], 50.0f, 0);
        }
    }

    rt_mutex_release(mutex_can_tx);
    return TRAJ_OK;
}

/* ---- Quintic 平滑插值 (v2: Commit B) ---- */
/*
 * s = 10u³ - 15u⁴ + 6u⁵
 * 边界条件: s(0)=0, s(1)=1, s'(0)=s'(1)=s''(0)=s''(1)=0
 * 即: 起点/终点速度=0, 加速度=0 → 无冲击
 */
static inline float quintic_smooth(float u)
{
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    float u2 = u * u;
    float u3 = u2 * u;
    return (10.0f - 15.0f * u + 6.0f * u2) * u3;
}

/*
 * 发送当前插值点到 CAN (逐关节, mode=0 轨迹跟踪)
 * 发送前执行安全检查: ESTOP / 错误状态 / 在线 / 零位 / 限位
 * 返回 0=成功, -1=安全拒绝
 */
static int _traj_send_interp_point(struct traj_interp_state *state)
{
    /* 急停检查 */
    if (g_sys.estop_triggered) {
        rt_kprintf("[Traj] ESTOP during interpolation, abort\n");
        return -1;
    }
    /* 错误状态 (允许 ERR_NOT_CALIBRATED 非致命) */
    if (g_sys.error_code != ERR_NONE && g_sys.error_code != ERR_NOT_CALIBRATED) {
        rt_kprintf("[Traj] Error 0x%02X during interpolation, abort\n",
                   g_sys.error_code);
        return -1;
    }
    /* 在线/零位检查 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (state->joint_skip[i]) continue;
        if (!g_sys.joint_online[i]) {
            rt_kprintf("[Traj] J%d offline during interpolation, abort\n", i + 1);
            return -1;
        }
        if (!g_sys.zero_valid[i]) {
            rt_kprintf("[Traj] J%d zero lost during interpolation, abort\n", i + 1);
            return -1;
        }
    }

    /* 发送 CAN (逐关节, mode=0 轨迹跟踪, param=50 滤波带宽) */
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (state->joint_skip[i]) continue;
        /* mode=0: 轨迹跟踪, speed=0 (不使用速度参数), param=50 (滤波带宽) */
        set_angle((uint8_t)(CAN_ID_JOINT_1 + i), state->q_raw[i], 0.0f, 50.0f, 0);
    }
    rt_mutex_release(mutex_can_tx);

    return 0;
}

/*
 * 每 10ms 调用一次: 计算 Quintic 插值点, 发送 CAN, 检查到位
 * 返回: 0=进行中, 1=段到位, -1=错误 (中止, 调用方应清理 FIFO)
 */
int traj_interp_tick(struct traj_interp_state *state)
{
    if (!state->active) return 1;

    state->elapsed_ms += TRAJ_INTERPOLATION_MS;

    /* 段结束: 发送最终目标值 */
    if (state->elapsed_ms >= state->duration_ms) {
        for (int i = 0; i < MAX_JOINT_COUNT; i++) {
            if (state->joint_skip[i]) continue;
            state->q_raw[i] = calib_model_to_raw(&g_calib, i + 1, state->q_target[i]);
        }
        int ret = _traj_send_interp_point(state);
        state->active = 0;
        return (ret < 0) ? -1 : 1;
    }

    /* 中间点: Quintic 插值 */
    float u = (float)state->elapsed_ms / (float)state->duration_ms;
    float s = quintic_smooth(u);

    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (state->joint_skip[i]) continue;
        float q_model = state->q_start[i] + s * (state->q_target[i] - state->q_start[i]);
        state->q_raw[i] = calib_model_to_raw(&g_calib, i + 1, q_model);
    }

    return _traj_send_interp_point(state);
}

/* ---- 到位检测 (纯函数, joint_ctrl 线程调用) ---- */
int traj_all_arrived(const float current_deg[MAX_JOINT_COUNT],
                     const float target_deg[MAX_JOINT_COUNT],
                     const uint8_t online[MAX_JOINT_COUNT])
{
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (!online[i]) continue;  /* 离线关节跳过 */
        float err = current_deg[i] - target_deg[i];
        if (err < 0.0f) err = -err;
        if (err > TRAJ_TOLERANCE_DEG)
            return 0;
    }
    return 1;
}
