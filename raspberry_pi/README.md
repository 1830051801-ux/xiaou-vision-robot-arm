# Raspberry Pi 侧工程

此目录是小U Python 工程根目录。运行脚本、模型、ROS 2 工作区和测试均以此为相对路径基准。

## 依赖分组

| 文件 | 适用场景 |
| --- | --- |
| `requirements.txt` | 小U基础交互、配置和算法工具 |
| `requirements_pi_inference.txt` | 2 GB Pi 的 ONNX CPU 推理；OpenCV 由 Raspberry Pi OS 提供 |
| `requirements_desktop_sim.txt` | 台式机训练、MuJoCo、PyTorch 和桌面仿真 |
| `requirements-dev.txt` | 测试与 CI |

## 常用离线入口

```bash
python -m unittest discover -s tests -v
python tools/verify_six_axis_stack.py
python tools/protocol_offline_replay.py
python tools/simulate_physical_grasp.py
python tools/ros2_offline_replay.py
```

### 物理仿真说明

`tools/simulate_physical_grasp.py` 默认只使用仓库内的 ROS 2 碰撞网格、桌面与刚体物体模型，因此干净克隆也能完成离线回归。若本机另有完整 CAD 世界模型，可显式追加 `--world-urdf <robot_world_model.urdf>` 进行网格引用审计；该外部 CAD 资产体积很大，不属于 GitHub 发布包。仿真仍以当前候选关节限位和 ready 位为准，若没有可行解，应先重新标定 TCP/桌面与限位，而不是放宽真实硬件配置。

这些脚本的设计目标是生成报告、轨迹预览或协议回放。请先阅读 `--help`，尤其不要把含 `--execute` 的工具用于未完成现场确认的设备。

## Pi 推理发布包

`tools/build_pi_deployment.py` 会从明确的白名单创建小型推理包：只包含 ONNX、INT8 策略、协议/状态代码和界面资源，不包含 PyTorch、训练集、MuJoCo 或硬件启用配置。构建过程不 SSH 到树莓派。
