# XiaoU 离线验证增量（2026-09-22）

本目录收录本轮重新运行的数值仿真证据。运行环境是本地 Windows/MuJoCo；`real_can_opened=false`、`real_motion_enabled=false`，没有打开树莓派、串口、CAN、相机或真实机械臂。

## 文件说明

| 文件 | 内容 |
| --- | --- |
| `desktop_scenarios_upgrade_20260920.json` | 瓶子、杯子、笔、纸巾四类桌面场景的 YOLO 契约、目标几何和动作候选回放 |
| `six_axis_pick_cad_tcp_upgrade_20260920.json` | 六轴离线取放路径；模型 TCP 姿态下五段路径均收敛 |
| `six_axis_pick_current_rpy_upgrade_20260920.json` | 当前 RPY 姿态对照；接近、下降、抬升段未收敛，作为问题证据保留 |
| `real_world_batch_upgrade_20260920.json` | 64 次带噪声数值回放；工作空间门限与旧 homography 存在配置不一致 |

## 解读边界

- CAD TCP 结果是模型对照，不是实机 TCP 标定结果；不能直接写入生产 ROS 2 参数。
- 数值 IK 没有替代 MoveIt PlanningScene 的碰撞检查，也没有证明电机跟踪能力。
- 64 次批处理全部在工作空间门限处被拒绝，说明下一步应先复测 homography、桌面坐标系和 TCP 姿态。
- `desktop_scenarios` 中的抓取成功是离线候选接受，不等于夹爪实际闭合或物体真实被提起。

这些 JSON 与对应脚本一起用于复现和问题定位，不应作为新的实机验收记录。
