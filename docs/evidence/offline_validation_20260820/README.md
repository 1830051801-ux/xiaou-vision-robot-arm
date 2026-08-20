# 离线验证记录（2026-08-20）

本目录记录了提交 `3e9969a` 在 Windows 本机完成的一次可复现离线验证。
所有结果均来自源码、协议回放、数值仿真或 GitHub Actions；**未连接树莓派、串口、CAN、相机或真实机械臂，不能作为实机验收记录。**

## 本次执行的命令

```powershell
py -3.13 -B scripts\verify_release.py
cd raspberry_pi
py -3.13 -B -m unittest discover -s tests -q
py -3.13 -B tools\verify_six_axis_stack.py
py -3.13 -B tools\protocol_offline_replay.py
py -3.13 -B tools\ros2_offline_replay.py
py -3.13 -B tools\simulate_six_axis_pick.py --orientation-mode cad_tcp --ik-restarts 5
```

## 结果摘要

| 项目 | 实际结果 | 边界 |
| --- | --- | --- |
| 发布结构检查 | 通过；857 个受跟踪文件，Keil 引用、模型登记和默认运动锁均通过 | 静态检查 |
| Python 单元测试 | 130 项通过，22.328 秒 | 纯软件测试 |
| 六轴模型一致性 | POE/URDF 最大螺旋轴误差 `3.97e-09`，Home 变换误差 `1.00e-12`；8 个视觉网格和 8 个碰撞网格存在 | 模型与文件验证，不是尺寸实测 |
| UART/CAN 回放 | 分片帧、CRC 拒绝、错误节点拒绝、J5 过期分类、J6 离线分类及运动锁均通过 | 合成帧；未打开串口或 SocketCAN |
| ROS 2 / Transformer 回放 | 六类桌面场景的 46 维观测契约通过；硬件门始终返回 `blocked_by_safety_gate` | 未启动 ROS 2 进程，未下发动作 |
| 六轴取放数值仿真 | 在合成杯子检测、假设限位和 `cad_tcp` 诊断姿态下，`approach → descend → lift → place → return_home` 均收敛 | 不是 TCP 标定、碰撞实测或真机抓取 |

## 如实记录的限制

- 当前 `RPY(pi, 0, yaw)` 目标姿态在同一数值仿真中，对 `approach`、`descend`、`lift` 三段均未收敛；它没有被隐藏或改写为成功。
- `cad_tcp` 路径只作为模型对齐诊断；未经现场 TCP 姿态、零位、方向、限位、桌面高度和抓取高度测量，不能复制到真实执行。
- 默认硬件配置仍保持运动锁定。报告没有把 J5/J6 的合成新鲜度案例写成设备在线状态。

## 机器可读证据

为使记录在公开仓库中可移植，`ros2_replay.json` 的检查点路径已由本机绝对路径归一为仓库相对路径；其余字段是上述命令的输出。

| 文件 | GitHub 文件（LF 规范化）SHA-256 |
| --- | --- |
| `six_axis_stack.json` | `0D34D6941E5D0FBEDD2B44B8416C9D940417F743C77BCD1330C2EF3670E03338` |
| `protocol_replay.json` | `54367596C1A2F35157E13A4EB036C3507DC54610861B37B26426B6C4872E3056` |
| `ros2_replay.json` | `F193D23BC14720510D0455D65D77B1C952E85AA02952A2722CD2523BC6ECB804` |
| `six_axis_pick_simulation.json` | `45226ADFDDAB17334FB183BA10BE975757F4D4B45088AC4F74A378168E605503` |

GitHub Actions 对同一发布分支的 `offline-verify` 工作流也已通过 `python-tests` 与 `release-layout` 两项检查。
