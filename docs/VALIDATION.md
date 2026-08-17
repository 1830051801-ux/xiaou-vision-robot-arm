# 离线验证与 CI

公开仓库只承诺可复现的软件验证；它不把仿真、协议回放或历史设备记录表述为新的实机验收。

## 本地验证

在 `raspberry_pi/` 目录执行：

```bash
python -m pip install -r requirements-dev.txt
python -m compileall -q robot_ai tools simulation tests
python -m unittest discover -s tests -v
python tools/verify_six_axis_stack.py
```

单元测试覆盖以下关键路径：

- 六轴 POE 运动学、IK、关节限位与五次轨迹；
- UART 帧、CRC、转义、状态/标定/轨迹点解码；
- CAN 编解码和无 SocketCAN 的回环模型；
- 关节回读新鲜度、离线特征和传输恢复判定；
- 抓取族、示教记录、预抓/抬升/转运/下降规划；
- YOLO 模型登记、哈希与类别模式；
- Transformer 观测契约、负样本和安全门；
- ROS 2 消息契约、状态面板与 Pi 部署包结构。

## 发布结构检查

仓库根目录的命令：

```bash
python scripts/verify_release.py
```

检查内容包括：必要源码和模型资产、Keil 工程中的源文件引用、公开仓库中不应出现的本地配置/构建产物，以及运行时资产是否仍处于锁定配置。

## GitHub Actions

`.github/workflows/offline-verify.yml` 会在 push 与 pull request 时执行：

1. 安装 Python 离线验证依赖；
2. 编译 Python 源码并运行测试；
3. 检查发布文件结构与 Keil 工程引用。

工作流不接触 GPIO、串口、CAN、相机或真实机械臂。

## 验证边界

- MuJoCo 物理回归默认使用仓库内的 ROS 2 碰撞网格；完整 CAD 世界模型属于可选的大型外部资产，只有显式传入 `--world-urdf` 时才审计其网格引用。当前候选限位、TCP 和桌面几何不满足时，脚本会报告无可行解，不会自行放宽真实硬件参数。
- 通过 Python 测试表示源码和离线契约一致，不代表已在当前设备上烧录并运动。
- ROS 2 工作区按源码发布，仍需要对应 ROS 2 发行版、MoveIt 和系统依赖来完成 `colcon build`。
- STM32 Keil 项目随源码发布，编译、下载、急停和机械限位确认应在现场单独记录。
