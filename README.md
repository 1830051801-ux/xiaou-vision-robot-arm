# XiaoU Vision Robot Arm Assistant

面向中国大学生智能装备创新大赛的视觉机械臂助手项目。系统目标是让桌面机械臂具备“看见物体、转换坐标、规划动作、发送控制命令、记录调试过程”的最小闭环能力。

## 项目亮点

- 视觉抓取链路：摄像头图像 -> YOLO 检测类别与 bbox -> 目标中心点 -> 标定转换 -> 机械臂坐标 -> IK / 控制命令 -> 夹爪动作。
- 树莓派运行环境：提供摄像头检测、语音/对话、小 U 桌宠界面、串口测试、STM32 联调脚本。
- 安全执行思路：在接入真实机械臂前，先使用 dry-run、角度限幅、串口命令打印和日志验证，降低误动作风险。
- 数据闭环扩展：可记录图像、检测框、机器人状态、动作、夹爪状态、成功/失败标签，用于后续 PyTorch 模仿学习策略训练。
- 竞赛展示结构清晰：视觉识别、坐标标定、运动学求解、执行安全、任务演示五个模块可独立讲解和演示。

## 演示视频

演示视频位于：

```text
media/demo.mp4
```

如果 GitHub 网页无法直接预览，可以下载后本地播放。

## 系统结构

```text
camera
-> YOLO detector
-> target adapter
-> pixel / workspace calibration
-> robot coordinate
-> IK or motion command
-> STM32 / serial dry-run
-> grasp or demo log
```

## 目录说明

```text
robot_ai/       主要 Python 程序：检测、对话、串口、表情屏、机械臂控制辅助
scripts/        树莓派安装、运行、测试、标定和联调脚本
docs/           使用说明、STM32 协议、相机标定、联调文档
models/         YOLO / ONNX 模型文件
media/          项目演示视频
config.*.env    配置模板，真实密钥不要提交
```

## 快速运行

在树莓派项目目录中运行：

```bash
cd ~/raspi_robot_ai
chmod +x scripts/*.sh
bash scripts/run_demo_all.sh
```

常用单项测试：

```bash
bash scripts/run_system_check.sh
bash scripts/run_camera_test.sh
bash scripts/run_yolo_test.sh
bash scripts/run_uart_test.sh
bash scripts/run_workspace_9point.sh
```

## 配置说明

复制配置模板并填写本机参数：

```bash
cp config.env.example config.env
```

不要把真实 `config.env`、API key、树莓派密码、WiFi 密码提交到仓库。

## 和具身智能训练的关系

本项目先把“感知 -> 坐标 -> 控制命令”跑通。后续可以把每次抓取过程记录成 episode：

```text
image, bbox, class_name, robot_x, robot_y, hand_state, action, gripper_state, success, failure_reason
```

这些数据可以接入 PyTorch 行为克隆、ACT / Diffusion Policy / VLA-style 策略训练，用真实抓取数据改进机械臂决策。

## 当前状态

- 已完成树莓派端视觉/对话/表情/串口相关程序组织。
- 已整理 STM32 串口联调和机械臂抓取协议文档。
- 已加入项目演示视频。
- 后续重点：实机抓取日志采集、成功率统计、PyTorch 策略接入、MuJoCo / Isaac Sim 仿真数据增强。
