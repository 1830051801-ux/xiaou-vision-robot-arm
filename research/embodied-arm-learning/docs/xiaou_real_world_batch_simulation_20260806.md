# XiaoU 真实条件多次离线仿真与 ROS2 全局审查

日期：2026-08-06  
项目：`raspi_robot_ai`

## 1. 仿真边界

这是基于真实项目模型和真实配置的离线压力仿真，不是实机运行。没有打开 `can0`，没有发送 CAN 控制帧，没有启动真实硬件控制，也没有解除四项运动安全锁。

仿真加入了：

- 5 帧目标中心观测和 1.5 px 像素噪声；
- 单应性平均残差 0.83 mm；
- 2 mm 目标放置误差；
- 3 deg 目标偏航误差；
- 抓取高度 3 mm 标准差；
- 标定凸包门禁、运行时工作区门禁、IK、速度/加速度轨迹限制；
- 100 个随机目标，固定随机种子 `20260806`，可重复。

## 2. 第一组：保持生产工作区门禁

结果文件：`runtime/simulations/real_world_batch_20260806.json`

```text
trials                         = 100
outside_calibrated_image      = 6
outside_robot_workspace       = 94
accepted_for_ik               = 0
current_algorithm_success     = 0
model_aligned_success         = 0
```

单应性输出范围约为：

```text
x = 254.2 .. 280.5 mm
y = -243.4 .. -216.2 mm
```

但运行时工作区配置是：

```text
X: 80 .. 360 mm
Y: -180 .. 180 mm
```

因此当前标定区域整体落在 `Y` 工作区之外。不是 IK 失败，而是感知目标在进入 IK 前就被正确拒绝。这个配置矛盾必须由真实基座坐标和工作台测量解决，不能通过放宽软件门禁掩盖。

## 3. 第二组：仅诊断，临时绕过工作区门禁

结果文件：`runtime/simulations/real_world_batch_geometry_only_20260806.json`

这组只用于把“工作区配置问题”和“机械姿态问题”分离，生产 ROS2 代码没有关闭门禁。

```text
trials                         = 100
outside_calibrated_image      = 3
accepted_for_ik               = 97

current RPY(pi,0,yaw):
  success                     = 0 / 97
  IK failures                 = 303 segment failures

model-aligned diagnostic pose:
  success                     = 95 / 97
  success rate                = 97.94%
  IK failures                 = 2 segment failures
```

结论很明确：

1. 当前生产 TCP 姿态 `RPY(pi,0,yaw)` 与六轴模型末端坐标定义不一致；
2. POE、FK、IK 和轨迹规划在模型对齐姿态下具备较高离线成功率；
3. 当前主要问题是工作区标定配置和 TCP 姿态标定，不是 CAN 回环或基础 IK 算法。

## 4. ROS2 架构改进

### 已完成

- `pipeline.launch.py` 增加 `start_perception` 参数；
- 新增 `review_only.launch.py`，默认不启动 YOLO/相机，不启动 CAN 硬件；
- MoveIt `allow_trajectory_execution=false` 保持硬关闭；
- planner `allow_execution=false` 保持硬关闭；
- planner 仍要求 `/xiaou/hardware_ready=true` 才能执行；
- 感知节点在发布目标前增加 `is_reachable()` 工作区门禁；
- 新增 `verify_ros2_architecture.py` 静态审计，检查节点层、话题、执行门禁和 review-only 隔离。

### 当前链路

```text
camera/Image -> YOLO/稳定帧 -> 单应性 -> 工作区门禁
             -> target_pose(base_link) -> MoveIt/IK/碰撞规划
             -> trajectory preflight -> hardware_ready gate
             -> ros2_control/CAN（当前锁定）
```

### 进程级启动验证

- `safety_smoke.launch.py` 已启动 `robot_state_publisher`、`joint_state_publisher` 和硬件就绪门禁；未启动相机、MoveIt 或 CAN，运行正常。
- `review_only.launch.py` 的项目配置错误已修复：补齐 `joint_limits.yaml`、review-only controller map，并将 OMPL 配置更新为 Jazzy 的数组格式。
- 当前 WSL 没有安装 `ros-jazzy-moveit-planners-ompl`，且 ROS/Ubuntu 软件源网络不可达，因此完整 MoveIt review-only 进程仍会在加载 `ompl_interface/OMPLPlanner` 时停止。这是环境依赖阻塞，不是项目代码静默失败；安装该包后应重新运行 review-only 启动测试。

## 5. 全局优化顺序

1. 重新测量 `base_link` 到工作台的坐标方向，重新生成单应性；确认 `Y` 正负号和单位，直到标定点落入真实工作区。
2. 用法兰孔和夹爪实体测量 `grasp_tcp` 的 X/Y/Z 轴，重新定义视觉输出姿态；不能直接把模型对齐姿态写进生产代码。
3. 为每个物品实测抓取高度，填写 `object_grasp_profiles.json`；未填写时继续锁定发布。
4. 单轴低速标定真实 ID、零位、方向、软/硬限位、速度和加速度。
5. 在 ROS2 `review_only.launch.py` 下运行 MoveIt PlanningScene 碰撞检查，再做双轴和六轴仿真。
6. STM32 完成 XiaoU CAN V1 回环后，才允许被动监听真实总线；最后才进入有人急停的单轴实测。

## 6. 验证

```text
Python tests: 24 passed
compileall: PASS
ROS2 architecture audit: PASS
ROS2 Jazzy build: 6 packages finished
ROS2 safety smoke: PASS
real_can_opened: false
real_motion_enabled: false
```

本报告中的成功率只代表带明确假设的离线模型试验，不能当作真实机械臂成功率。
