# 部署

公开仓库面向 Raspberry Pi 5 8 GB + STM32F407VGT6 的分层部署。Pi 运行视觉、任务编排、策略回放和 UART 桥接；F407 负责实时解析、轨迹队列和 CAN 关节控制。

## 离线安装

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/verify_release.py
```

ROS 2/MoveIt 需要目标发行版的系统依赖，见 [`ROS2.md`](ROS2.md)。模型文件和配置必须与注册表中的 SHA/类别表匹配。

## 分阶段上机

1. 在断电或运动锁定状态检查配置、固件哈希、线束和急停。
2. 只读验证 UART PING、状态帧、六轴反馈新鲜度和 CRC。
3. 运行视觉/标定预览和轨迹 dry-run，不发送电机动作。
4. 由现场负责人确认零位、限位、速度、加速度和夹爪参数后，才进入单轴低速验证。
5. 完成空载、轻载和任务回放记录，再启用完整任务链路。

公开 `hardware_calibration.json` 保持 `motion_enabled: false`。站点配置不得提交到公共仓库。
