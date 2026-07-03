#!/usr/bin/env python3
"""
机械臂 3D 实时可视化 + 轨迹仿真 + 交互控制
===========================================
用法:
    python arm_viewer.py               # 交互模式 (默认)
    python arm_viewer.py COM3          # 串口实时
    python arm_viewer.py --plan "j1,j2,j3,j4"

特性:
    - 立体圆柱连杆 + 关节球 + 地面网格 + 阴影
    - 鼠标左键旋转 / 右键缩放 / 中键平移视角
    - 绿色目标点 + 灰色路径预览 + 实心机械臂动画
    - 底部控制面板: J1-J4 输入 + PLAN/RESET/HOME 按钮
    - 右侧实时曲线: 关节角度 + 速度剖面 (S曲线)

依赖: pip install pyserial numpy matplotlib
"""
import sys, math, time, argparse, threading
from collections import deque

import numpy as np
import matplotlib
# 尝试设置中文字体, 失败则回退
try:
    for _f in ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC']:
        try:
            matplotlib.font_manager.findfont(_f, fallback_to_default=False)
            matplotlib.rcParams['font.sans-serif'] = [_f, 'DejaVu Sans']
            matplotlib.rcParams['axes.unicode_minus'] = False
            break
        except Exception:
            continue
except Exception:
    pass
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa

from traj_planner import (TrajPlanner, traj_set_joint_ptp, traj_start,
                          traj_generate_waypoints)

# ==========================================================================
# 常量
# ==========================================================================
DH = [
    {"d": 300.0, "a":   0.0, "alpha": math.radians(90.0), "off": 0.0},
    {"d":   0.0, "a": 180.0, "alpha": math.radians( 0.0), "off": 0.0},
    {"d":   0.0, "a": 100.0, "alpha": math.radians(90.0), "off": 0.0},
    {"d":   0.0, "a":   0.0, "alpha": math.radians( 0.0), "off": 0.0},
]
LIMITS = [(-160, 160), (-120, 0), (-60, 120), (-120, 120)]
HOME   = [0.0, -45.0, 90.0, 0.0]
SPEED, ACCEL, JERK = 60.0, 120.0, 600.0

# 机械臂视觉参数
LINK_RADII  = [18, 14, 12, 8, 5]   # 连杆半径 (mm), 越远端越细
CYL_SEG  = 8   # 圆柱边数 (少=快)
SPH_SEG  = 6   # 球体分辨率 (少=快)
JOINT_COLOR = '#FF6D00'             # 关节球颜色 (橙色)
LINK_COLOR  = '#1565C0'             # 连杆颜色 (深蓝)
TIP_COLOR   = '#D50000'             # 末端
TARGET_CLR  = '#00C853'             # 目标点
GHOST_CLR   = '#B0BEC5'             # 路径预览
GROUND_CLR  = '#ECEFF1'             # 地面
JOINT_COLORS = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12']  # J1-J4 曲线颜色

# ==========================================================================
# 逆运动学 (匹配 MCU arm_kinematics.c 解析解)
# ==========================================================================
def ik(pose, current_joints=None):
    """
    4轴机械臂逆运动学解析解
    pose: [x(mm), y(mm), z(mm), roll(°), pitch(°), yaw(°)]
    current_joints: 当前关节角度[4] (°), 用于多解择优
    返回: [j1, j2, j3, j4] (°) 或 None (不可达)
    """
    x, y, z, roll, pitch, yaw = pose
    d1, a2, a3 = DH[0]["d"], DH[1]["a"], DH[2]["a"]

    # — θ1: 底座旋转 —
    theta1 = math.degrees(math.atan2(y, x))

    # — 投影到 XZ 平面 —
    r_xy = math.sqrt(x*x + y*y)
    zh = z - d1

    # — 2 连杆 IK (θ2, θ3) —
    cos_theta3 = (r_xy*r_xy + zh*zh - a2*a2 - a3*a3) / (2.0 * a2 * a3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3_up   = math.degrees(math.acos(cos_theta3))   # 肘向上
    theta3_down = -theta3_up                              # 肘向下

    def _solve_theta2(t3):
        """由 θ3 算 θ2 (相对水平面)"""
        psi = math.atan2(zh, r_xy)
        phi = math.atan2(a3 * math.sin(math.radians(t3)),
                         a2 + a3 * math.cos(math.radians(t3)))
        return math.degrees(psi - phi)

    theta2_up   = _solve_theta2(theta3_up)
    theta2_down = _solve_theta2(theta3_down)

    # — 多解择优 (选最接近 current_joints 的解) —
    if current_joints is not None:
        candidates = [
            (theta2_up,   theta3_up),
            (theta2_down, theta3_down),
        ]
        best, best_dist = (theta2_up, theta3_up), 1e9
        for t2, t3 in candidates:
            dist = (abs(t2 - current_joints[1]) + abs(t3 - current_joints[2]))
            if dist < best_dist:
                best_dist = dist
                best = (t2, t3)
        theta2, theta3 = best
    else:
        theta2, theta3 = theta2_down, theta3_down  # 默认肘向下

    # — θ4: 手腕旋转 = yaw - sum(θ1..θ3) 的水平分量 —
    theta4 = yaw  # 简化: 直接使用 yaw (在实际4轴中手腕独立旋转)

    # — 限位检查 —
    for i, (v, (lo, hi)) in enumerate(zip(
        [theta1, theta2, theta3, theta4], LIMITS)):
        if v < lo or v > hi:
            return None  # 超出关节限位

    return [theta1, theta2, theta3, theta4]


# ==========================================================================
# 运动学
# ==========================================================================
def dh_mat(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ])

def fk(joints):
    """返回 6 个关键点 (底座,J1,J2,J3,腕,末端工具)"""
    pts = np.zeros((5, 3))
    T = np.eye(4)
    for i, (ang, dh) in enumerate(zip(joints, DH)):
        th = math.radians(ang) + dh["off"]
        T = T @ dh_mat(th, dh["d"], dh["a"], dh["alpha"])
        pts[i + 1] = T[:3, 3]
    tool = T[:3, 3] + T[:3, 2] * 230.0
    return np.vstack([pts, tool.reshape(1, 3)])

# ==========================================================================
# 3D 几何 (预计算模板 + 平移复用, 避免每帧重算)
# ==========================================================================

# ── 球体模板缓存 ──
_SPHERE_CACHE = {}  # (seg) -> (unit_verts, faces)

def _sphere_template(seg=SPH_SEG):
    """返回 r=1 中心=(0,0,0) 的单位球面模板 (verts, faces), 带缓存"""
    if seg in _SPHERE_CACHE:
        return _SPHERE_CACHE[seg]
    phi = np.linspace(0, np.pi, seg)
    th  = np.linspace(0, 2 * np.pi, seg)
    x = np.outer(np.sin(phi), np.cos(th))
    y = np.outer(np.sin(phi), np.sin(th))
    z = np.outer(np.cos(phi), np.ones_like(th))
    verts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    faces = []
    for i in range(seg - 1):
        for j in range(seg - 1):
            a = i * seg + j; b = a + 1; c = a + seg; d = c + 1
            faces += [[a, b, d], [a, d, c]]
    _SPHERE_CACHE[seg] = (verts, faces)
    return verts, faces

def _sphere_at(center, radius, seg=SPH_SEG):
    """返回平移到 center 并缩放到 radius 的球体顶点 (复用模板)"""
    uv, faces = _sphere_template(seg)
    return uv * radius + np.array(center), faces

# ── 圆柱模板缓存 ──
_CYL_CACHE = {}  # (seg) -> (unit_circle_2d, face_indices)

def _cyl_template(seg=CYL_SEG):
    if seg in _CYL_CACHE:
        return _CYL_CACHE[seg]
    th = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    circle = np.column_stack([np.cos(th), np.sin(th)])  # (seg, 2)
    faces = []
    for i in range(seg):
        j = (i + 1) % seg
        faces += [[i, j, seg + i], [j, seg + j, seg + i]]
    _CYL_CACHE[seg] = (circle, faces)
    return circle, faces

def _cylinder_pairs(pts, radii, seg=CYL_SEG):
    """返回 [(verts, faces)] — 每段圆柱, 复用圆模板"""
    circle, faces = _cyl_template(seg)
    result = []
    for k in range(len(pts) - 1):
        p1, p2 = pts[k], pts[k + 1]
        r = radii[k]
        v = p2 - p1
        length = np.linalg.norm(v)
        if length < 1e-6:
            continue
        vn = v / length
        ref = np.array([1, 0, 0]) if abs(vn[0]) < 0.9 else np.array([0, 1, 0])
        u = np.cross(vn, ref); u /= np.linalg.norm(u)
        w = np.cross(vn, u)
        # 用 2D 圆模板生成 3D 圆环
        circ3d = r * (np.outer(circle[:, 0], u) + np.outer(circle[:, 1], w))
        bottom = p1 + circ3d
        top   = p2 + circ3d
        result.append((np.vstack([bottom, top]), faces))
    return result

# ── 夹爪几何 (两指平行板) ──
def _gripper_geometry(tip, wrist, grip_width=18.0):
    """
    返回夹爪两根手指的 [(verts, faces), (verts, faces)]
    tip/wrist: FK 输出的末端点和腕关节点
    grip_width: 两指向外侧总宽度 (mm)
    """
    v = tip - wrist
    length = np.linalg.norm(v)
    if length < 1e-6:
        return [np.zeros((4, 3)), np.zeros((4, 3))], [[0, 1, 2], [0, 2, 3]]
    vn = v / length  # 工具方向 (进近)

    # 垂直于 vn 的夹爪开合方向
    ref = np.array([1, 0, 0]) if abs(vn[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(vn, ref); u /= np.linalg.norm(u)

    fl = 12.0  # 指长 (沿工具反方向)
    fw = 3.0   # 指宽/厚
    fd = grip_width * 0.5  # 半开度

    results = []
    for side in [-1.0, 1.0]:
        # 手指矩形: 4 个顶点 (tip 附近, 指根 → 指尖, 偏移 ±u)
        center = tip + u * (side * fd)
        p0 = center + u * fw - vn * 2       # 指根外侧
        p1 = center - u * fw - vn * 2       # 指根内侧
        p2 = center - u * fw - vn * (2 + fl)  # 指尖内侧
        p3 = center + u * fw - vn * (2 + fl)  # 指尖外侧
        verts = np.array([p0, p1, p2, p3])
        results.append(verts)

    faces = [[0, 1, 2], [0, 2, 3]]
    return results, faces

# ==========================================================================
# 串口
# ==========================================================================
class SerialReader:
    def __init__(self, port, baud=115200):
        self.port, self.baud = port, baud
        self.queue = deque(maxlen=100)
        self.running, self.ser = False, None

    def start(self):
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"[SERIAL] {self.port} @ {self.baud}")

    def _run(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._parse(line)
            except Exception:
                time.sleep(0.1)

    def _parse(self, line):
        try:
            s = line.decode("utf-8", errors="ignore").strip()
            if s.startswith("J:"):
                parts = s[2:].split(",")
                if len(parts) >= 4:
                    self.queue.append([float(p) for p in parts[:4]])
        except (ValueError, IndexError):
            pass

    def get(self):
        if self.queue:
            data = None
            while self.queue:
                data = self.queue.popleft()
            return data
        return None

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

# ==========================================================================
# 优化 3D 渲染器
# ==========================================================================
class OptimizedViewer:
    """立体机械臂 + 控制面板 — 圆柱连杆/关节球/地面/阴影"""

    def __init__(self, serial=None):
        self.serial = serial
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.target = [45.0, -30.0, 80.0, 15.0]
        self.speed, self.accel = SPEED, ACCEL

        self.tp = TrajPlanner()
        self.waypoints, self.wp_idx = [], 0
        self.plan_active = False
        self.anim_t = 0.0

        self.hist_t, self.hist_j, self.hist_v = [], [[] for _ in range(4)], [[] for _ in range(4)]

        # ---- figure 布局 ----
        self.fig = plt.figure(figsize=(16, 9), facecolor='#FAFAFA')
        self.fig.canvas.manager.set_window_title("4-Axis Robot Arm — Optimized 3D")

        # 3D (左侧大)
        self.ax3d = self.fig.add_axes([0.02, 0.15, 0.55, 0.83], projection='3d')
        self._init_3d()
        self._init_3d_artists()

        # 关节曲线 (右上)
        self.ax_j = self.fig.add_axes([0.60, 0.56, 0.38, 0.40])
        self._init_joint_plot()

        # 速度曲线 (右下)
        self.ax_v = self.fig.add_axes([0.60, 0.15, 0.38, 0.35])
        self._init_vel_plot()

        # 控制面板 (底部)
        self._init_controls()

        # 信息表格 (顶部, 深色底)
        self.txt_info = self.fig.text(0.02, 0.965, "", fontsize=8,
                                       fontfamily="monospace", va='top',
                                       bbox=dict(boxstyle='round,pad=0.4',
                                                 facecolor='#263238', alpha=0.88),
                                       color='#ECEFF1')

        self._update_info()
        self.ani = animation.FuncAnimation(self.fig, self._update, interval=40,
                                            blit=False, cache_frame_data=False)
        plt.show()

    # ---- 3D 初始化 ----
    def _init_3d(self):
        ax = self.ax3d
        ax.set_xlim3d(-500, 500); ax.set_ylim3d(-500, 500); ax.set_zlim3d(0, 650)
        ax.set_xlabel("X mm"); ax.set_ylabel("Y mm"); ax.set_zlabel("Z mm")
        ax.set_title("4-Axis Robot Arm", fontsize=12, fontweight='bold', pad=20)
        ax.view_init(elev=25, azim=-60)  # 初始视角
        # 地面网格
        gx = np.linspace(-500, 500, 11)
        gy = np.linspace(-500, 500, 11)
        gz = np.zeros((11, 11))
        ax.plot_surface(*np.meshgrid(gx, gy), gz, alpha=0.08, color=GROUND_CLR,
                         zorder=0, rasterized=True)
        for v in gx:
            ax.plot([v, v], [-500, 500], [0, 0], 'k-', lw=0.3, alpha=0.15)
            ax.plot([-500, 500], [v, v], [0, 0], 'k-', lw=0.3, alpha=0.15)
        # 坐标轴指示
        ax.plot([0, 80], [0, 0], [0, 0], 'r-', lw=2)
        ax.plot([0, 0], [0, 80], [0, 0], 'g-', lw=2)
        ax.plot([0, 0], [0, 0], [0, 80], 'b-', lw=2)

    def _init_3d_artists(self):
        """预创建合并的大 collection, 减少 set_verts 调用次数"""
        # ── 各连杆独立 collection (视觉好, shade=True) ──
        pts = fk(self.joints)
        cyls = _cylinder_pairs(pts, LINK_RADII)
        self.link_meshes = []
        for verts, faces in cyls:
            m = Poly3DCollection(verts[faces], facecolors=LINK_COLOR, alpha=0.92,
                                  shade=True, linewidth=0, zorder=5)
            self.ax3d.add_collection3d(m)
            self.link_meshes.append(m)

        # ── 关节球 (J1-J4, 跳过 pts[0]=原点) ──
        self.sphere_meshes = []
        for i in range(4):
            sp = pts[i + 1]   # pts[1]..pts[4] = J1..J4
            r = LINK_RADII[min(i + 1, len(LINK_RADII)-1)] + 4
            sv, sf = _sphere_at(sp, r, SPH_SEG)
            m = Poly3DCollection(sv[sf], facecolors=JOINT_COLORS[i],
                                  alpha=0.95, shade=True, linewidth=0, zorder=6)
            self.ax3d.add_collection3d(m)
            self.sphere_meshes.append(m)

        # ── 夹爪 (两指) ──
        self.gripper_meshes = []
        gv_list, gf = _gripper_geometry(pts[-1], pts[-2], 18.0)
        for gv in gv_list:
            m = Poly3DCollection(gv[gf], facecolors='#546E7A', alpha=0.92,
                                  shade=True, linewidth=0, zorder=7)
            self.ax3d.add_collection3d(m)
            self.gripper_meshes.append(m)

        # ── 目标 ──
        tgt_pts = fk(self.target)
        tgv, tgf = _sphere_at(tgt_pts[-1], 20, SPH_SEG)
        self.target_mesh = Poly3DCollection(tgv[tgf], facecolors=TARGET_CLR, alpha=0.35,
                                             linewidth=0, zorder=4)
        self.ax3d.add_collection3d(self.target_mesh)

        # ── 底座 ──
        bv, bf = _sphere_at([0, 0, DH[0]["d"]/2], 35, 10)
        self.base_mesh = Poly3DCollection(bv[bf], facecolors='#455A64', alpha=0.9,
                                           shade=True, linewidth=0, zorder=3)
        self.ax3d.add_collection3d(self.base_mesh)

        # ── 路径预览线 ──
        (self.ghost_line,) = self.ax3d.plot([], [], [], '-', lw=1.5,
                                             color=GHOST_CLR, alpha=0.5, zorder=2)

    # ---- 关节 & 速度图 ----
    def _init_joint_plot(self):
        ax = self.ax_j
        ax.set_title("Joint Angles", fontsize=10, fontweight='bold')
        ax.set_ylabel("Angle (°)")
        ax.grid(True, alpha=0.25)
        ax.set_xlim(0, 5); ax.set_ylim(-180, 180)
        self.j_lines = []
        for c in JOINT_COLORS:
            (l,) = ax.plot([], [], color=c, lw=2, alpha=0.9)
            self.j_lines.append(l)
        (self.j_cursor,) = ax.plot([], [], 'k--', lw=1, alpha=0.5)

    def _init_vel_plot(self):
        ax = self.ax_v
        ax.set_title("Velocity Profile", fontsize=10, fontweight='bold')
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (°/s)")
        ax.grid(True, alpha=0.25)
        ax.set_xlim(0, 5); ax.set_ylim(-100, 100)
        self.v_lines = []
        for c in JOINT_COLORS:
            (l,) = ax.plot([], [], color=c, lw=1.5, alpha=0.8)
            self.v_lines.append(l)

    # ---- 控制面板 (plt.axes 精确定位) ----
    def _init_controls(self):
        left, bottom = 0.03, 0.03
        cw, gh, ah = 0.048, 0.015, 0.035

        def _ax(col, row=0):
            return plt.axes([left + col * (cw + gh),
                             bottom + row * (ah + 0.012),
                             cw, ah])

        # ====== Row 0: 关节角度控制 ======
        defaults = ['45.0', '-30.0', '80.0', '15.0']
        self.tb_j1 = TextBox(_ax(0), 'J1:', defaults[0], textalignment="center")
        self.tb_j2 = TextBox(_ax(1), 'J2:', defaults[1], textalignment="center")
        self.tb_j3 = TextBox(_ax(2), 'J3:', defaults[2], textalignment="center")
        self.tb_j4 = TextBox(_ax(3), 'J4:', defaults[3], textalignment="center")
        self._tbs = [self.tb_j1, self.tb_j2, self.tb_j3, self.tb_j4]

        # 标签颜色匹配关节曲线颜色
        for tb, clr in zip(self._tbs, JOINT_COLORS):
            tb.label.set_color(clr)
            tb.label.set_fontweight('bold')

        off = 5
        self.tb_spd = TextBox(_ax(off), 'Spd:', '60', textalignment="center")
        self.tb_acc = TextBox(_ax(off+1), 'Acc:', '120', textalignment="center")

        bw = 0.055
        btn_off = off + 3
        def _btn_ax(n):
            return plt.axes([left + (btn_off + n) * (cw + gh) + n * (bw - cw),
                             bottom, bw, ah * 1.2])

        self.btn_plan = Button(_btn_ax(0), 'PLAN', color='#4CAF50', hovercolor='#A5D6A7')
        self.btn_plan.on_clicked(self._plan)
        self.btn_reset = Button(_btn_ax(1), 'ZERO', color='#EF5350', hovercolor='#EF9A9A')
        self.btn_reset.on_clicked(self._zero)
        self.btn_home = Button(_btn_ax(2), 'HOME', color='#FFCA28', hovercolor='#FFE082')
        self.btn_home.on_clicked(self._home)
        # 视角预设按钮 (紧凑排列)
        vw = 0.026  # 半宽按钮
        vg = 0.004
        def _vax(n):
            return plt.axes([left + (btn_off + 3) * (cw + gh) + n * (vw + vg),
                             bottom, vw, ah * 1.2])

        self.btn_view = Button(_vax(0), '3D', color='#42A5F5', hovercolor='#90CAF9')
        self.btn_view.on_clicked(self._view_reset)
        self.btn_vf = Button(_vax(1), 'F', color='#66BB6A', hovercolor='#A5D6A7')
        self.btn_vf.on_clicked(self._view_front)
        self.btn_vs = Button(_vax(2), 'S', color='#FFA726', hovercolor='#FFCC80')
        self.btn_vs.on_clicked(self._view_side)
        self.btn_vt = Button(_vax(3), 'T', color='#AB47BC', hovercolor='#CE93D8')
        self.btn_vt.on_clicked(self._view_top)

        # ====== Row 1: 树莓派仿真 (笛卡尔 → IK → 关节) ======
        row1_bottom = bottom + ah + 0.014
        def _ax1(col):
            return plt.axes([left + col * (cw + gh), row1_bottom, cw, ah])

        self.tb_x  = TextBox(_ax1(0), 'X:',  '200', textalignment="center",
                              color='#E8F5E9')
        self.tb_y  = TextBox(_ax1(1), 'Y:',  '100', textalignment="center",
                              color='#E8F5E9')
        self.tb_z  = TextBox(_ax1(2), 'Z:',  '-50', textalignment="center",
                              color='#E8F5E9')
        self.tb_th = TextBox(_ax1(3), 'θ:',  '0', textalignment="center",
                              color='#E8F5E9')
        self.tb_zs = TextBox(_ax1(4), 'Zs:', '50', textalignment="center",
                              color='#FFF3E0')

        self._pi_tbs = [self.tb_x, self.tb_y, self.tb_z, self.tb_th, self.tb_zs]

        # Pi 按钮
        pi_btn_off = 6
        def _pi_btn_ax(n):
            return plt.axes([left + (pi_btn_off + n) * (cw + gh) + n * (bw - cw),
                             row1_bottom, bw, ah * 1.2])

        self.btn_ik = Button(_pi_btn_ax(0), 'SEND', color='#00BCD4',
                              hovercolor='#80DEEA')
        self.btn_ik.on_clicked(self._pi_send)

        self.btn_proto = Button(_pi_btn_ax(1), 'PROTO', color='#78909C',
                                 hovercolor='#B0BEC5')
        self.btn_proto.on_clicked(self._pi_proto)

        # 协议帧预览文本
        self.txt_proto = self.fig.text(0.03, row1_bottom - 0.018,
            "", fontsize=7, fontfamily="monospace", va='top', color='#546E7A')

    # ---- 按钮 ----
    def _plan(self, event):
        try:
            self.target = [float(tb.text) for tb in self._tbs]
            self.speed = float(self.tb_spd.text)
            self.accel = float(self.tb_acc.text)
        except ValueError:
            print("[ERROR] Invalid number"); return
        for i, (v, (lo, hi)) in enumerate(zip(self.target, LIMITS)):
            if v < lo or v > hi:
                print(f"[WARN] J{i+1}={v} out of [{lo}, {hi}]")

        traj_set_joint_ptp(self.tp, self.target, self.speed, self.accel, JERK)
        traj_start(self.tp, self.joints)
        self.waypoints = traj_generate_waypoints(self.tp, interval_ms=25)
        self.wp_idx = 0
        self.plan_active = True

        self.hist_t.clear(); self.hist_j = [[] for _ in range(4)]
        self.hist_v = [[] for _ in range(4)]
        T = self.tp.total_duration_ms / 1000.0 + 0.5
        self.ax_j.set_xlim(0, T); self.ax_v.set_xlim(0, T)

        for ln in list(self.ax_j.lines):
            if getattr(ln, '_mark', False): ln.remove()
        for i in range(4):
            l1 = self.ax_j.axhline(self.target[i], color=f'C{i}', ls=':', alpha=0.4); l1._mark = True
            l2 = self.ax_j.axhline(self.joints[i], color=f'C{i}', ls=':', alpha=0.15); l2._mark = True

        print(f"[PLAN] -> {self.target}  {self.tp.total_duration_ms}ms")

    def _zero(self, event):
        for tb in self._tbs: tb.set_val("0.0")
        self._plan(event)

    def _home(self, event):
        for i, tb in enumerate(self._tbs): tb.set_val(str(HOME[i]))
        self._plan(event)

    def _view_reset(self, event):
        """重置 3D 视角到默认斜视"""
        self.ax3d.view_init(elev=25, azim=-60)
        self.ax3d.set_xlim3d(-500, 500)
        self.ax3d.set_ylim3d(-500, 500)
        self.ax3d.set_zlim3d(0, 650)

    def _view_front(self, event):
        """正面视角 (XZ平面)"""
        self.ax3d.view_init(elev=5, azim=-90)
        self.ax3d.set_xlim3d(-500, 500)
        self.ax3d.set_ylim3d(-500, 500)
        self.ax3d.set_zlim3d(0, 650)

    def _view_side(self, event):
        """侧面视角 (YZ平面)"""
        self.ax3d.view_init(elev=5, azim=0)
        self.ax3d.set_xlim3d(-500, 500)
        self.ax3d.set_ylim3d(-500, 500)
        self.ax3d.set_zlim3d(0, 650)

    def _view_top(self, event):
        """俯视视角"""
        self.ax3d.view_init(elev=89, azim=0)
        self.ax3d.set_xlim3d(-500, 500)
        self.ax3d.set_ylim3d(-500, 500)
        self.ax3d.set_zlim3d(0, 650)

    # ---- 树莓派仿真 (笛卡尔 → IK → PLAN) ----
    def _pi_send(self, event):
        """模拟树莓派发送笛卡尔坐标 → IK → 关节 → PLAN"""
        try:
            x   = float(self.tb_x.text)
            y   = float(self.tb_y.text)
            z   = float(self.tb_z.text)
            yaw = float(self.tb_th.text)
            zs  = float(self.tb_zs.text)
        except ValueError:
            print("[PI SIM] Invalid input"); return

        pose = [x, y, z, 0.0, 0.0, yaw]
        result = ik(pose, self.joints)
        if result is None:
            print(f"[PI SIM] IK FAILED: X={x} Y={y} Z={z} θ={yaw}° 不可达!")
            self.txt_proto.set_text(
                f"❌ IK FAILED: ({x}, {y}, {z}, θ={yaw}°) 超出工作空间")
            return

        for i, tb in enumerate(self._tbs):
            tb.set_val(f"{result[i]:.1f}")
        self.txt_proto.set_text(
            f"✅ Pi → IK OK: X={x} Y={y} Z={z} θ={yaw}° → "
            f"J=[{result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f}, {result[3]:.1f}]  "
            f"Zsafe={zs}mm")
        print(f"[PI SIM] X={x} Y={y} Z={z} θ={yaw}° → "
              f"J=[{result[0]:.1f} {result[1]:.1f} {result[2]:.1f} {result[3]:.1f}]")

        self._plan(event)

    def _pi_proto(self, event):
        """显示模拟的二进制协议帧 (CMD_MOVE_CART)"""
        try:
            x   = float(self.tb_x.text)
            y   = float(self.tb_y.text)
            z   = float(self.tb_z.text)
            yaw = float(self.tb_th.text)
            zs  = float(self.tb_zs.text)
        except ValueError:
            self.txt_proto.set_text("❌ PROTO: Invalid number"); return

        import struct
        payload = struct.pack('<hhhhhBB',
            int(x * 10), int(y * 10), int(z * 10),
            int(yaw * 10), int(zs * 10), 0x00, 0x00)
        crc = _crc16_modbus(b'\x12' + bytes([len(payload)]) + b'\x01' + payload)
        frame = bytes([0xAA, 0x12, len(payload), 0x01]) + payload + struct.pack('<H', crc) + bytes([0x55])

        self.txt_proto.set_text(
            f"📡 CMD_MOVE_CART ({len(frame)}B):  "
            f"{frame.hex(' ').upper()}\n"
            f"   X={x:.1f}mm Y={y:.1f}mm Z={z:.1f}mm θ={yaw:.1f}° Zsafe={zs:.1f}mm  "
            f"CRC={crc:04X}")

    # ---- 更新循环 ----
    def _update(self, frame):
        if self.serial:
            d = self.serial.get()
            if d: self.joints = d
        elif self.plan_active and self.wp_idx < len(self.waypoints):
            _, self.joints = self.waypoints[self.wp_idx]
            self.wp_idx += 1

        t_now = (self.wp_idx * 0.025) if self.plan_active else 0.0

        # 记录历史
        if self.plan_active:
            self.hist_t.append(t_now)
            for i in range(4):
                self.hist_j[i].append(self.joints[i])
                if len(self.hist_t) > 1:
                    dt = t_now - self.hist_t[-2]
                    self.hist_v[i].append((self.joints[i] - self.hist_j[i][-2]) / max(dt, 1e-6))

        # ── 3D: 逐 collection 更新 set_verts (几何缓存复用) ──
        pts = fk(self.joints)
        cyls = _cylinder_pairs(pts, LINK_RADII)
        for mesh, (verts, faces) in zip(self.link_meshes, cyls):
            mesh.set_verts(verts[faces])

        for i, mesh in enumerate(self.sphere_meshes):
            v, f = _sphere_at(pts[i + 1], LINK_RADII[min(i + 1, 4)] + 4, SPH_SEG)
            mesh.set_verts(v[f])

        # 夹爪更新
        gv_list, gf = _gripper_geometry(pts[-1], pts[-2], 18.0)
        for mesh, gv in zip(self.gripper_meshes, gv_list):
            mesh.set_verts(gv[gf])

        tgt_pts = fk(self.target)
        tgv, tgf = _sphere_at(tgt_pts[-1], 20, SPH_SEG)
        self.target_mesh.set_verts(tgv[tgf])

        # 路径预览
        if self.plan_active and self.waypoints:
            all_tips = np.array([fk(w[1])[-1] for w in self.waypoints[::2]])
            self.ghost_line.set_data(all_tips[:, 0], all_tips[:, 1])
            self.ghost_line.set_3d_properties(all_tips[:, 2])

        # ── 曲线 -─
        if self.plan_active and len(self.hist_t) >= 2:
            for i in range(4):
                self.j_lines[i].set_data(self.hist_t, self.hist_j[i])
            self.j_cursor.set_data([t_now, t_now], self.ax_j.get_ylim())
            if t_now > self.ax_j.get_xlim()[1] - 0.4:
                self.ax_j.set_xlim(0, t_now + 1.5)
            for i in range(4):
                if len(self.hist_v[i]) >= 2:
                    n = len(self.hist_v[i])
                    self.v_lines[i].set_data(self.hist_t[1:1+n], self.hist_v[i])
            if t_now > self.ax_v.get_xlim()[1] - 0.4:
                self.ax_v.set_xlim(0, t_now + 1.5)
            if self.hist_j[0]:
                all_j = [v for jh in self.hist_j for v in jh]
                if all_j:
                    mn, mx = min(all_j) - 5, max(all_j) + 5
                    if mx > mn: self.ax_j.set_ylim(mn, mx)

        self._update_info()
        return []

    def _update_info(self):
        pts = fk(self.joints)
        tip = pts[-1]
        pct = min(self.wp_idx / max(len(self.waypoints), 1), 1.0) * 100 if self.plan_active else 0

        status = f"PLAN {int(pct)}%" if self.plan_active else "READY"
        self.txt_info.set_text(
            " Joint Angles (deg)  |  End-Effector (mm)   |  Status\n"
            "----------------------+-----------------------+--------\n"
            f" J1={self.joints[0]:+6.1f} J2={self.joints[1]:+6.1f}"
            f" | X={tip[0]:+7.1f} Y={tip[1]:+7.1f}"
            f" | {status}\n"
            f" J3={self.joints[2]:+6.1f} J4={self.joints[3]:+6.1f}"
            f" | Z={tip[2]:+7.1f}"
            "              |"
        )

# ==========================================================================
# CRC16 / 协议辅助
# ==========================================================================
def _crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS (匹配 MCU proto_crc16)"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ==========================================================================
# CLI 入口
# ==========================================================================
def main():
    parser = argparse.ArgumentParser(description="4-Axis Robot Arm 3D Viewer")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--plan", type=str, default=None)
    parser.add_argument("--from", type=str, default="0,0,0,0", dest="fr")
    parser.add_argument("--speed", type=float, default=SPEED)
    parser.add_argument("--accel", type=float, default=ACCEL)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    if args.plan:
        target = [float(x.strip()) for x in args.plan.split(",")]
        start  = [float(x.strip()) for x in args.fr.split(",")]
        if len(target) != 4 or len(start) != 4:
            print("Need 4 angles"); sys.exit(1)
        tp = TrajPlanner()
        traj_set_joint_ptp(tp, target, args.speed, args.accel, JERK)
        traj_start(tp, start)
        wps = traj_generate_waypoints(tp, 20)
        print(f"Duration: {tp.total_duration_ms}ms, waypoints: {len(wps)}")
        print(f"Dominant: J{tp.dominant_joint+1}, phases(ms): {tp.phase_duration}")
        for t, j in wps[::max(1, len(wps)//10)]:
            print(f"  t={t:4d}ms  J=[{', '.join(f'{x:6.1f}' for x in j)}]")
        print(f"  t={wps[-1][0]:4d}ms  J=[{', '.join(f'{x:6.1f}' for x in wps[-1][1])}]  <- DONE")
        return

    sr = None
    if args.port:
        try:
            sr = SerialReader(args.port, args.baud)
            sr.start()
        except Exception as e:
            print(f"[ERROR] Serial {args.port}: {e}"); sr = None

    try:
        OptimizedViewer(serial=sr)
    except KeyboardInterrupt:
        print("\n[DONE]")
    finally:
        if sr: sr.stop()

if __name__ == "__main__":
    main()
