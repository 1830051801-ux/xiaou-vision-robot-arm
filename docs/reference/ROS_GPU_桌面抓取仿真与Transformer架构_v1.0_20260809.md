# ROS2 + GPU 桌面抓取仿真与 Transformer 决策架构 v1.0

## 当前主线

现有相机链路保持不变：

```text
相机帧
  -> YOLO ONNX 检测类别/框/中心点
  -> 多帧稳定与置信度检查
  -> 固定相机单应性 pixel -> base_link
  -> ROS /xiaou/target_pose + /xiaou/target_status
  -> Transformer 选择任务策略/阶段
  -> MoveIt IK、工作空间、碰撞和轨迹检查
  -> hardware_ready 安全门
  -> 真实控制器（当前永远关闭）
```

Transformer 只做任务级决策，不直接产生 CAN、PWM 或六关节角度。它的输出是
`strategy`、`phase`、候选抓取参数和风险值；ROS/MoveIt 和硬件安全门仍是确定性
验证层。

## 已加入的离线场景

`simulation/desktop_scene.py` 提供五个可复现任务：

- `bottle`：侧向包覆抓取；
- `cup`：侧向包覆抓取；
- `pen`：小物体顶部夹持；
- `desktop_item`：普通桌面盒状物平抓；
- `tissue_pull`：抓住纸巾边缘后沿桌面方向抽取，禁止把纸巾堆当刚体抬起。

这一步是任务级几何仿真，用于验证消息、策略和失败分支；它不是已经完成的接触动力学仿真。
当前 Windows 环境没有 ROS2、Gazebo、MuJoCo 或 PyBullet 运行时，因此不会把它误称为真实
物理结果。后续接入 Gazebo/MuJoCo 时，保持同一 observation/action contract，不改变安全门。

运行五类离线场景：

```powershell
cd D:\机械臂\work\raspi_robot_ai_safety_20260808
python simulation\run_desktop_scenarios.py --scenarios bottle,cup,pen,desktop_item,tissue_pull --output runtime\simulations\desktop_scenarios_20260809.json
```

## ROS2 话题

离线场景节点：

```bash
ros2 launch xiaou_arm_simulation desktop_scene.launch.py scenario:=bottle
```

完整 ROS 评审入口（MoveIt、规划和真实执行默认关闭）：

```bash
ros2 launch xiaou_arm_planning simulation_pipeline.launch.py \
  project_root:=/home/pi/raspi_robot_ai_transport_only_20260808 \
  scenario:=bottle \
  checkpoint:=runtime/decision/transformer_policy.pt
```

它发布：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/xiaou/sim/observation` | `std_msgs/String` JSON | YOLO 风格观测、物体几何、关节快照和安全状态 |
| `/xiaou/sim/action_candidates` | `std_msgs/String` JSON | 规则基线候选，用来比较 Transformer 输出 |
| `/xiaou/sim/target_pose` | `geometry_msgs/PoseStamped` | 仿真目标位姿，独立于真实 `/xiaou/target_pose` |
| `/xiaou/sim/status` | `std_msgs/String` JSON | 仿真阶段和候选应用结果 |
| `base_link -> sim_<object>` | TF | RViz 中查看桌面物体位置 |

将策略应用到仿真状态只需发布 JSON 到 `/xiaou/sim/strategy`；这不会触发 MoveIt 或硬件。

## Transformer

模型输入是长度最多 16 的 46 维观测序列：YOLO 中心/置信度、目标 `base_link` 位姿、
物体几何和质量、6 轴位置/速度/在线位、任务阶段、夹爪状态和安全标志。

模型输出：

- `strategy`：`top_down_pinch`、`side_wrap`、`pen_pinch`、`flat_pick`、`tissue_pull` 或 `reject`；
- `phase`：观察、接近、抓取、抬升、抽取或中止；
- 6 个候选动作参数；
- `risk` 风险标量。

初始训练标签来自可解释的场景规则，仅用于验证 GPU/ROS 数据链路，不代表真实抓取能力。
真实训练必须采集同步的 YOLO 观测、目标位姿、关节反馈、夹爪状态、接触结果和失败原因。

训练入口：

```powershell
cd D:\机械臂\work\raspi_robot_ai_safety_20260808
..\.venv_transformer_gpu\Scripts\python.exe robot_ai\decision\train_transformer_policy.py --device cuda
```

默认输出：

```text
runtime/decision/transformer_policy.pt
runtime/decision/transformer_training.json
```

ROS 决策节点：

```bash
ros2 run xiaou_arm_decision decision_node --ros-args \
  -p project_root:=/home/pi/raspi_robot_ai_transport_only_20260808 \
  -p checkpoint:=runtime/decision/transformer_policy.pt \
  -p simulation_mode:=true
```

没有 checkpoint、torch、有效观测或真实硬件门状态时，节点只发布 `blocked`，不发布可执行
控制命令。真实相机接入时，`decision_observation_bridge` 从已有 `/xiaou/target_status`、
`/xiaou/target_pose`、`/joint_states` 和 `/xiaou/hardware_ready` 组装同一观测格式；在
`simulation_mode:=false` 下，未验证的反馈/急停/硬件状态会强制阻断。

## 目前不作出的结论

现阶段不能声称瓶子、水杯、笔或纸巾已经在真实机械臂上成功抓取；也不能把合成策略标签
当作真实数据。下一阶段必须先解决真实 TCP 姿态、桌面高度、每类物体抓取高度/夹爪参数、
碰撞场景和六轴反馈，之后再用相机录制真实示范并替换合成训练集。
