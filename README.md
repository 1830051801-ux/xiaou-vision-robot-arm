# 小U（XiaoU）视觉具身桌面机器人

小U是一个坐标驱动的六轴桌面机器人项目，覆盖视觉感知、相机标定、抓取规划、树莓派软件、STM32F407 控制器、CAN 关节接口和 ROS 2 离线验证。仓库目标是把一条真实可拆解的机器人工程链路整理成可测试、可回放、可逐步上机的工程，而不是把一个动作脚本包装成完整系统。

> 默认状态是离线验证和运动锁定：不会自动打开相机、串口或 CAN，也不会由测试和示例命令发送真实运动指令。真实上机必须由现场人员按硬件前置条件逐项确认。

## 系统链路

~~~text
相机图像
  -> YOLO / ONNX 检测
  -> 相机标定与像素坐标转换
  -> 目标坐标与抓取姿态预览
  -> 树莓派 UART（序号、转义、CRC16、ACK）
  -> STM32F407 状态机与轨迹 FIFO
  -> CAN 六轴关节
  -> 关节反馈、新鲜度检查与状态展示
~~~

任务由目标类别、像素/桌面坐标、抓取配置和轨迹数据驱动，不依赖“1 号拿水、2 号放置”这类固定任务编号。视觉结果也不会直接越过标定、边界和控制器校验变成电机命令。

## 工程内容

| 层级 | 主要内容 |
| --- | --- |
| 视觉 | OpenCV、YOLO/ONNX 模型注册、类别白名单、检测结果新鲜度和坐标处理 |
| 标定与规划 | 桌面单应性/坐标变换、六轴 POE 正运动学、阻尼最小二乘 IK、预抓/抓取/抬升/放置轨迹 |
| 树莓派 | 观测、任务编排、抓取族、Transformer 回放接口、离线仿真、UART 协议和状态面板 |
| ROS 2 | 机器人描述、MoveIt 配置、规划预览、状态接口和离线回放 |
| F407 控制器 | Keil 工程、RT-Thread 线程、UART 帧解析、轨迹队列、CAN 收发、六轴反馈和急停状态 |
| 验证 | 协议回放、模型一致性检查、六轴数值仿真、单元测试和发布结构检查 |

## 离线验证证据

以下数据来自 2026-08-20 Windows 本机的源码检查、合成协议回放和数值仿真；没有连接树莓派、相机、串口、CAN 或真实机械臂，不能当作实机验收数据。

| 检查项 | 记录结果 | 结论边界 |
| --- | ---: | --- |
| Python 单元测试 | 130 项通过 | 纯软件测试 |
| 六轴模型一致性 | POE/URDF 最大螺旋轴误差 3.97e-09；Home 变换误差 1.00e-12 | 模型文件与计算一致性，不是机械尺寸实测 |
| UART/CAN 回放 | 分片、CRC 错误、错误节点、J5 过期、J6 离线和运动锁检查通过 | 合成帧，没有打开硬件接口 |
| ROS 2 / Transformer 回放 | 46 维观测契约通过，硬件门保持 blocked_by_safety_gate | 未启动 ROS 2 进程，未下发动作 |
| 六轴取放数值仿真 | approach -> descend -> lift -> place -> return_home 收敛 | 不等于 TCP 标定、碰撞实测或真机抓取 |

机器可读结果和完整限制见 [离线验证记录](docs/evidence/offline_validation_20260820/README.md)。

## 快速开始：仅离线

需要 Python 3.10+。以下命令不会打开硬件接口：

~~~powershell
git clone https://github.com/1830051801-ux/xiaou-vision-robot-arm.git
cd xiaou-vision-robot-arm
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -U pip
.\.venv\Scripts\python.exe scripts\verify_release.py
Push-Location raspberry_pi
..\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
..\.venv\Scripts\python.exe -m unittest discover -s tests -q
..\.venv\Scripts\python.exe tools\verify_six_axis_stack.py
..\.venv\Scripts\python.exe tools\protocol_offline_replay.py
Pop-Location
~~~

在 Linux/Raspberry Pi 上，将 Python 命令替换为系统 Python 或虚拟环境解释器。ROS 2 和 MoveIt 运行时需要额外的发行版依赖，详见 [ROS 2 文档](docs/ROS2.md)。

## 目录

~~~text
raspberry_pi/
  robot_ai/                 感知、坐标变换、抓取规划、UART 和交互
  tools/                    离线回放、仿真、部署与诊断工具
  tests/                    纯软件单元测试和协议契约测试
  simulation/               桌面场景与策略数据接口
  ros2_ws/src/              ROS 2、MoveIt 和机器人描述源码
  models/                   选定的视觉模型与类别表
  runtime/                  少量可复现运行时资产和离线证据

stm32_keil/
  BasicSetting_DaRanRobot/  STM32F407 + RT-Thread + CAN + Keil 工程

docs/                       架构、验证、上机边界和协议参考
scripts/                    发布结构检查
media/                      演示素材（发布前单独审查体积和来源）
~~~

## 真实硬件边界

公开仓库中的硬件配置保持运动锁定。更换设备后，至少需要重新确认：

1. 相机内外参、桌面高度、坐标系方向和单位；
2. 六个关节的零位、方向、限位、速度/加速度上限和 CAN 节点响应；
3. Pi-F407 UART 接线、波特率、帧版本、CRC、ACK 和超时策略；
4. F407 固件哈希、急停链路、反馈新鲜度和轨迹队列行为；
5. 低速单轴、空载、无夹具、低风险范围内的分阶段验证记录。

teach_grasp_execute.py 的执行入口与离线预览分开，并要求显式参数和现场确认；它不属于 README 的演示流程。未经测量的抓取高度、零偏和限位不能从示例配置复制到另一台机械臂。

## 文档入口

- [系统架构](docs/ARCHITECTURE.md)
- [离线验证与 CI](docs/VALIDATION.md)
- [ROS 2 与离线仿真](docs/ROS2.md)
- [模型与量化](docs/MODELS.md)
- [硬件上机与联调](docs/HARDWARE_BRINGUP.md)
- [STM32F407 / Keil 工程](stm32_keil/README.md)
- [公开发布范围](docs/PUBLICATION.md)
- [第三方组件说明](THIRD_PARTY_NOTICES.md)

## 公开发布范围

不要把荣耀、富士康或其他内部项目的图片、日志、模型、源码、产品型号和标定数据放入本仓库。大模型、视频、部署压缩包和原始训练集应放在经过审查的 Release/LFS 资产中，源码提交只保留可复现且有来源说明的文件。具体规则见 docs/PUBLICATION.md。

## 许可

本仓库顶层代码按 LICENSE 的项目许可说明提供；STM32 CMSIS、HAL、RT-Thread 和其他第三方组件继续遵循各自目录中的原始许可证，详见 THIRD_PARTY_NOTICES.md。
