# XiaoU 六轴取物离线仿真结果

日期：2026-08-06  
工程：`raspi_robot_ai`

## 1. 仿真边界

本次仿真完全离线运行：没有打开 `can0`，没有启动 `can_control.launch.py`，没有发送真实 CAN 控制帧，也没有解除 `motion_enabled` 安全锁。

使用的是真实项目中的：

- `arm_model.json` 中的六轴 POE 模型；
- `workspace_homography.yaml` 中的 2D 像素到 `base_link` 单应性；
- 当前目标姿态生成规则；
- 当前 IK 和五次多项式轨迹规划器；
- XiaoU CAN V1 Python 协议参考实现。

由于抓取高度、关节限位和 TCP 实际姿态尚未测量，本次仅使用明确标记的仿真假设：

- 检测类别：`cup`；置信度：`0.92`；像素中心：`(318, 297)`；
- 桌面高度：`0.0 m`；抓取高度：`0.03 m`；
- 关节位置范围：`[-pi, +pi]`；速度上限：`0.4 rad/s`；加速度上限：`0.8 rad/s^2`。

这些假设没有写入生产标定配置。

## 2. 感知坐标结果

单应性将像素 `(318, 297)` 转换为：

```text
base_link x = 0.2685005 m
base_link y = -0.2302662 m
```

该点位于当前标定工作区和模型的水平工作半径内。

## 3. 当前生产姿态的结果

当前 `target_pose_node` 使用：

```text
RPY(pi, 0, yaw)
```

在上述目标点和仿真假设下，当前取物链没有形成完整轨迹：

| 段 | IK | 位置误差 | 姿态误差 | 结果 |
|---|---:|---:|---:|---|
| approach | 失败 | 0.1064 m | 0.1205 rad | 未生成轨迹 |
| descend | 失败 | 0.0886 m | 0.0502 rad | 未生成轨迹 |
| lift | 失败 | 0.1064 m | 0.1205 rad | 未生成轨迹 |
| place | 通过 | 8.18e-6 m | 2.41e-5 rad | 746 点，7.44 s |
| return_home | 通过 | 1.05e-6 m | 0 rad | 746 点，7.44 s |

当前生产算法的结论：`complete_pick_path=false`。

这不是可以忽略的仿真失败。它说明视觉节点输出的 TCP 姿态与当前机械模型的末端坐标定义不一致，不能直接让真实机械臂执行。

## 4. 姿态对齐诊断对比

为了区分姿态定义问题和运动学代码问题，仿真额外测试了仅用于诊断的姿态：

```text
Rz(yaw) * model.home_grasp_tcp.rotation
```

该姿态不是生产配置，只是模型一致性对照。结果：

| 段 | IK | 位置误差 | 姿态误差 | 轨迹 |
|---|---:|---:|---:|---:|
| approach | 通过 | 4.34e-6 m | 0 rad | 732 点，7.31 s |
| descend | 通过 | 4.07e-6 m | 0 rad | 182 点，1.81 s |
| lift | 通过 | 3.26e-6 m | 0 rad | 182 点，1.81 s |
| place | 通过 | 3.37e-5 m | 1.00e-4 rad | 142 点，1.41 s |
| return_home | 通过 | 1.59e-5 m | 3.60e-6 rad | 701 点，6.99 s |

对齐对比的结论：`complete_pick_path=true`。这证明当前 POE、FK、IK 和五次轨迹代码可以在模型一致的姿态下完成一条离线取物路径，但不能证明真实 TCP 姿态已经正确。

## 5. CAN V1 回环结果

协议回环没有使用 SocketCAN：

```text
socketcan_opened       = false
all_joints_reached     = true
quick_stop_ok          = true
watchdog_fault_ok      = true
J1..J6 positions       = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30] rad
```

回环验证了：

1. 六个节点的命令 ID、位置/速度缩放和序列号；
2. 周期命令维持 200 ms 看门狗；
3. quick-stop 后速度为零且关节禁用；
4. 停止发送命令后自动进入 fault。

## 6. 已发现的问题与修复顺序

### 必须优先修复

1. **TCP 姿态定义不一致**：用实物法兰/夹爪建立 `grasp_tcp` 的实际坐标轴，确认抓取方向，再修改视觉姿态生成规则或 TCP 固定变换。
2. **抓取高度仍为 null**：对 `cup`、`cola`、`bottle`、`pen`、`earphone` 分别实测 TCP 高度，未测量前保持目标发布锁定。
3. **关节限位和方向未实测**：不能把本次 `[-pi,+pi]` 假设写入生产配置。
4. **MoveIt 碰撞仿真尚未替代**：本脚本检查了数值 IK 和轨迹约束，但没有冒充 MoveIt PlanningScene 的碰撞检查；应在 ROS2 WSL/树莓派仿真环境做 review-only 规划场景验证。

### 已完成的安全边界

- `motion_enabled=false`；
- `protocol_confirmed=false`；
- `estop_verified=false`；
- `feedback_verified=false`；
- 未配置真实 CAN 接口和波特率；
- 未启动真实控制节点。

## 7. 复现命令

```powershell
cd "C:\Users\ZhuanZ（无密码）\Desktop\桌面整理_2026-07-01_033319\raspi_robot_ai"
py -m pytest -q tests
py tools\calibrate_six_axis.py --output runtime\simulations\calibration_draft_20260806.json
py tools\simulate_six_axis_pick.py --output runtime\simulations\six_axis_pick_20260806.json
```

结果文件：

- `runtime/simulations/six_axis_pick_20260806.json`
- `runtime/simulations/calibration_draft_20260806.json`

