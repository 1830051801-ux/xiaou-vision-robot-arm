# 小U树莓派侧使用说明

本目录是小U的 Python 工程根目录。它包含视觉、语音/表情、六轴运动学、示教轨迹、UART 协议、ROS 2 工作区和离线测试。

## 推荐先做离线验证

```bash
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests -v
python tools/verify_six_axis_stack.py
python tools/protocol_offline_replay.py
python tools/ros2_offline_replay.py
```

上述命令不会打开串口或发送机械臂动作。详细说明见同目录的 [README.md](README.md) 及仓库根目录 `docs/`。

## 树莓派推理环境

2 GB Raspberry Pi 只安装 `requirements_pi_inference.txt`：OpenCV 由 Raspberry Pi OS 提供，YOLO 与 Transformer 使用 ONNX Runtime CPU 推理。训练、MuJoCo 和 PyTorch 仅在桌面端使用 `requirements_desktop_sim.txt`。

## 硬件配置

提交的标定配置用于格式和离线回归，默认保持运动锁定。相机标定、零位、方向、限位和现场速度必须重新实测；不要复用历史相机标定或其他设备的数值。
