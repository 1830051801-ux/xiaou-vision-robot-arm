# ROS 2 与离线仿真

ROS 2 工作区位于 `raspberry_pi/ros2_ws/`，仅提交 `src/`。`build/`、`install/` 和 `log/` 都由本地 `colcon` 生成，不应提交。

## 包结构

| 包 | 作用 |
| --- | --- |
| `xiaou_arm_description` | 六轴 URDF/Xacro、视觉/碰撞网格、关节状态与显示启动文件 |
| `xiaou_arm_moveit_config` | MoveIt 配置、关节限制、规划器和控制器配置 |
| `xiaou_arm_perception` | 目标观测到 ROS 消息的桥接 |
| `xiaou_arm_decision` | Transformer 观测与决策消息契约 |
| `xiaou_arm_planning` | 目标规划、示教路点预览和离线启动文件 |
| `xiaou_arm_can_control` | CAN 控制接口配置与实现骨架 |
| `xiaou_arm_hardware` | 硬件接口骨架；不得在未确认的设备上启用 |
| `xiaou_arm_simulation` | 桌面场景节点和仿真消息源 |

## 构建示例

在已安装匹配 ROS 2 与 MoveIt 的 Linux 环境中：

```bash
cd raspberry_pi/ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

先使用 `review_only.launch.py`、`simulation_pipeline.launch.py` 或其他明确标为 preview/simulation 的启动文件验证 TF、话题和消息形状。它们用于查看模型、复现消息链路和离线规划，不构成实机启动授权。

## 模型与坐标

网格和模型状态与 Pi 侧 `arm_model.json`、六轴运动学实现共同构成离线参考。现实机械臂的零位、关节方向、限位和相机外参必须用实测结果覆盖；ROS 2 模型不能代替这一步。
