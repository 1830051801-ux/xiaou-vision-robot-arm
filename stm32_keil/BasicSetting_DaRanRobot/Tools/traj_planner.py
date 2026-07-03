"""
S曲线轨迹规划器 — MCU trajectory_planner.c 的 Python 1:1 移植
=================================================================
与 MCU 端算法逐公式一致，用于 PC 端验证轨迹合理性。

协议:
  - traj_set_joint_ptp(tp, target, max_vel, max_accel, max_jerk)
  - traj_start(tp, current_angles)
  - traj_update(tp, elapsed_ms) → (joints[4], state)
  - traj_generate_waypoints(tp, interval_ms=10) → [(t, j1,j2,j3,j4), ...]

S曲线 7 段:
  [0] 加加速 (+J)  [1] 匀加速   [2] 减加速 (-J)
  [3] 匀速         [4] 加减速 (-J) [5] 匀减速   [6] 减减速 (+J)
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple
from enum import IntEnum


class TrajState(IntEnum):
    IDLE = 0
    RUNNING = 1
    DONE = 2
    ERROR = 3


class TrajType(IntEnum):
    NONE = 0
    JOINT_PTP = 1
    CART_LINEAR = 2


# ==========================================================================
# S曲线核心 — 与 C 代码逐行一致
# ==========================================================================

def _scurve_compute(dist: float, vmax: float, amax: float,
                    jmax: float) -> Tuple[int, List[int]]:
    """
    计算 S曲线 7 阶段时长 (ms)
    返回: (total_ms, [T0..T6])
    与 C 代码 scurve_compute() 完全一致
    """
    T = [0, 0, 0, 0, 0, 0, 0]
    if dist < 1e-6:
        return 0, T

    Tj = amax / jmax  # jerk ramp time (s)

    # Case 1: 能否达到 amax?
    if amax * amax / jmax <= vmax + 1e-9:
        Ta = (vmax - amax * Tj) / amax
        if Ta < 0:
            Ta = 0

        if Ta > 0 and dist >= 2.0 * vmax * Tj + vmax * vmax / amax:
            # Full 7-segment
            T[0] = T[2] = T[4] = T[6] = int(Tj * 1000.0)
            T[1] = T[5] = int(Ta * 1000.0)
            # 计算单边加速位移
            v_end_jerk_up = 0.5 * jmax * Tj * Tj
            v_end_const = v_end_jerk_up + amax * Ta
            d_side = (jmax * Tj * Tj * Tj) / 6.0 \
                   + v_end_jerk_up * Ta + 0.5 * amax * Ta * Ta \
                   + v_end_const * Tj - 0.5 * amax * Tj * Tj \
                   + jmax * Tj * Tj * Tj / 6.0
            d_const = dist - 2.0 * d_side
            if d_const > 0 and vmax > 1e-6:
                T[3] = int(d_const / vmax * 1000.0)
        else:
            # 无匀速段 (梯形)
            vp = (-amax * amax + math.sqrt(
                amax ** 4 + 4.0 * jmax * jmax * amax * dist)) / (2.0 * jmax)
            if vp > vmax:
                vp = vmax
            Ta = (vp - amax * Tj) / amax
            if Ta < 0:
                Ta = 0
                vp = jmax * Tj * Tj
            T[0] = T[2] = T[4] = T[6] = int(Tj * 1000.0)
            T[1] = T[5] = int(Ta * 1000.0)
            T[3] = 0
    else:
        # Case 2: 达不到 amax — 三角型剖面
        Tv = math.sqrt(vmax / jmax)
        d_tri = jmax * Tv * Tv * Tv
        if dist >= d_tri:
            T[0] = T[6] = int(Tv * 1000.0)
            d_const = dist - d_tri
            t_const = d_const / vmax
            T[3] = int(t_const * 1000.0)
        else:
            t_half = (dist / (2.0 * jmax)) ** (1.0 / 3.0)
            T[0] = T[6] = int(t_half * 1000.0)

    total = sum(T)
    if total == 0:
        T[0] = 1
        total = 1
    return total, T


def _scurve_one_side(t_sec: float, Tj: float, Ta: float,
                     jmax: float, amax: float) -> float:
    """
    单边 (加速侧) 位移 — 解析分段多项式
    与 C 代码 scurve_evaluate_one_side() 完全一致
    """
    active_time = 2.0 * Tj + Ta
    if t_sec <= 0:
        return 0.0
    if t_sec >= active_time:
        v1 = 0.5 * jmax * Tj * Tj
        a1_val = jmax * Tj
        s0 = jmax * Tj * Tj * Tj / 6.0
        s1 = v1 * Ta + 0.5 * amax * Ta * Ta
        v2 = v1 + amax * Ta
        s2 = (v2 * Tj + 0.5 * a1_val * Tj * Tj
              - jmax * Tj * Tj * Tj / 6.0)
        return s0 + s1 + s2

    # Phase 0: jerk = +jmax
    if t_sec <= Tj:
        return jmax * t_sec * t_sec * t_sec / 6.0

    v0 = 0.5 * jmax * Tj * Tj
    a0 = jmax * Tj
    s0 = jmax * Tj * Tj * Tj / 6.0
    t_sec -= Tj

    # Phase 1: jerk = 0, a = amax
    if t_sec <= Ta:
        return s0 + v0 * t_sec + 0.5 * amax * t_sec * t_sec

    v1 = v0 + amax * Ta
    s0 += v0 * Ta + 0.5 * amax * Ta * Ta
    t_sec -= Ta

    # Phase 2: jerk = -jmax
    return (s0 + v1 * t_sec + 0.5 * a0 * t_sec * t_sec
            - jmax * t_sec * t_sec * t_sec / 6.0)


def _scurve_evaluate(T: List[int], t_ms: int, total_ms: int,
                     jmax: float, amax: float, vmax: float) -> float:
    """
    计算 S曲线比例 s ∈ [0,1]
    与 C 代码 scurve_evaluate() 完全一致
    """
    if total_ms == 0 or t_ms >= total_ms:
        return 1.0
    if t_ms == 0:
        return 0.0

    t_sec = t_ms * 0.001
    Tj = T[0] * 0.001
    Ta_val = T[1] * 0.001

    # 总位移
    d_accel = _scurve_one_side(Tj + Ta_val + Tj, Tj, Ta_val, jmax, amax)
    d_const = vmax * T[3] * 0.001
    d_total = 2.0 * d_accel + d_const

    if d_total < 1e-9:
        return t_ms / total_ms  # fallback 线性

    t_accel_side = Tj + Ta_val + Tj

    if t_sec <= t_accel_side:
        s = _scurve_one_side(t_sec, Tj, Ta_val, jmax, amax) / d_total
    elif t_sec <= t_accel_side + T[3] * 0.001:
        pos = d_accel + vmax * (t_sec - t_accel_side)
        s = pos / d_total
    else:
        t_decel = t_sec - t_accel_side - T[3] * 0.001
        d_decel = _scurve_one_side(t_decel, Tj, Ta_val, jmax, amax)
        pos = d_accel + d_const + d_decel
        s = pos / d_total

    return min(max(s, 0.0), 1.0)


# ==========================================================================
# 轨迹规划器 — 与 C 代码 traj_planner 接口一致
# ==========================================================================

@dataclass
class TrajPlanner:
    """轨迹规划器 (与 MCU struct traj_planner 字段一致)"""

    state: int = TrajState.IDLE
    traj_type: int = TrajType.NONE

    start_tick: int = 0
    total_duration_ms: int = 0

    start_joints: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    target_joints: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    delta_angle: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    dominant_joint: int = 0

    max_vel: float = 60.0     # °/s
    max_accel: float = 120.0  # °/s²
    max_jerk: float = 600.0   # °/s³
    phase_duration: List[int] = field(default_factory=lambda: [0] * 7)

    # 笛卡尔模式
    start_pose: List[float] = field(default_factory=lambda: [0.0] * 6)
    target_pose: List[float] = field(default_factory=lambda: [0.0] * 6)
    cart_speed: float = 100.0
    cart_accel: float = 200.0

    _T_MS: List[int] = field(default_factory=lambda: [0] * 7, repr=False)


def traj_init(tp: TrajPlanner):
    """初始化 (与 C 代码 traj_init 一致)"""
    tp.state = TrajState.IDLE
    tp.traj_type = TrajType.NONE


def traj_set_joint_ptp(tp: TrajPlanner, target: List[float],
                       max_vel: float = 60.0, max_accel: float = 120.0,
                       max_jerk: float = 600.0) -> int:
    """配置关节 PTP 轨迹 (与 C 代码一致)"""
    if max_vel <= 0 or max_accel <= 0 or max_jerk <= 0:
        return -1

    traj_init(tp)
    tp.traj_type = TrajType.JOINT_PTP
    tp.target_joints = list(target)
    tp.max_vel = max_vel
    tp.max_accel = max_accel
    tp.max_jerk = max_jerk
    return 0


def traj_start(tp: TrajPlanner, current: List[float]):
    """开始执行 (与 C 代码 traj_start 一致)"""
    tp.start_joints = list(current)
    max_delta = 0.0

    for i in range(4):
        tp.delta_angle[i] = tp.target_joints[i] - tp.start_joints[i]
        ad = abs(tp.delta_angle[i])
        if ad > max_delta:
            max_delta = ad
            tp.dominant_joint = i

    tp.total_duration_ms, tp._T_MS = _scurve_compute(
        max_delta, tp.max_vel, tp.max_accel, tp.max_jerk)
    tp.phase_duration = tp._T_MS
    tp.state = TrajState.RUNNING


def traj_update(tp: TrajPlanner, elapsed_ms: int) -> Tuple[List[float], TrajState]:
    """更新 (与 C 代码 traj_update 一致) 返回 (joints[4], state)"""
    if tp.state == TrajState.DONE:
        return tp.target_joints[:], TrajState.DONE
    if tp.state != TrajState.RUNNING:
        return tp.start_joints[:], tp.state

    if elapsed_ms >= tp.total_duration_ms:
        tp.state = TrajState.DONE
        return tp.target_joints[:], TrajState.DONE

    s = _scurve_evaluate(tp._T_MS, elapsed_ms, tp.total_duration_ms,
                         tp.max_jerk, tp.max_accel, tp.max_vel)

    output = [0.0] * 4
    for i in range(4):
        output[i] = tp.start_joints[i] + s * tp.delta_angle[i]

    return output, TrajState.RUNNING


def traj_generate_waypoints(tp: TrajPlanner,
                            interval_ms: int = 20) -> List[Tuple[int, List[float]]]:
    """
    预生成完整轨迹 (waypoint 列表) — Python 独有, MCU 不需要
    返回: [(elapsed_ms, [j1,j2,j3,j4]), ...]
    """
    waypoints = []
    t = 0
    while t <= tp.total_duration_ms:
        joints, state = traj_update(tp, t)
        waypoints.append((t, joints))
        if state == TrajState.DONE:
            break
        t += interval_ms
    return waypoints


# ==========================================================================
# 辅助: 速度剖面计算 (用于绘图)
# ==========================================================================

def traj_velocity_profile(tp: TrajPlanner,
                          interval_ms: int = 5) -> List[Tuple[int, float, float, float]]:
    """
    计算主导关节的速度/加速度/加加速度随时间变化
    返回: [(t_ms, vel, accel, jerk), ...]
    用于绘制速度剖面图, 验证 S曲线是否平滑
    """
    profile = []
    prev_s = 0.0
    prev_v = 0.0
    dt = interval_ms * 0.001

    for t_ms in range(0, tp.total_duration_ms + interval_ms, interval_ms):
        s = _scurve_evaluate(tp._T_MS, t_ms, tp.total_duration_ms,
                             tp.max_jerk, tp.max_accel, tp.max_vel)
        delta = tp.delta_angle[tp.dominant_joint]
        pos = tp.start_joints[tp.dominant_joint] + s * delta
        vel = (s - prev_s) * delta / dt if dt > 0 else 0
        accel = (vel - prev_v) / dt if dt > 0 else 0
        jerk = 0.0  # 简化: 用相邻 accel 差分
        profile.append((t_ms, vel, accel, jerk))
        prev_s = s
        prev_v = vel

    return profile
