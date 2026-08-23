# GPU YOLO 与 Transformer 离线链路

当前链路固定为：

```text
相机帧 -> UnifiedYolo -> 类别/置信度/框/中心点
       -> 2D 单应性与目标位姿
       -> 46 维 ROS 观测序列
       -> PyTorch Transformer -> 策略/阶段/有界动作候选
       -> SafetyGate 与 MoveIt 校验
       -> 硬件门（当前关闭）
```

## YOLO 模型

`robot_ai/vision/unified_yolo.py` 统一两种后端：

- `.pt`：Windows GPU 上用 Ultralytics/PyTorch CUDA；
- `.onnx`：树莓派和 Windows 回归测试用 OpenCV DNN CPU。

多模型用逗号分隔。默认第一模型为主模型，后续模型只补第一模型没有覆盖的类别；需要研究并集时加 `--union`。这样新四类模型不会把旧通用模型的低质量瓶子误报并入笔图。

当前本地数据只包含 `pen`、`bottle`、`cola`、`earphone`，没有 `cup` 或 `tissue` 标注。`cup` 只能由旧模型补充；纸巾目前只在仿真中使用 `tissue_pull` 策略，不能宣称新模型已经学会水杯或纸巾。

## GPU 训练与导出

```powershell
cd D:\机械臂\work\raspi_robot_ai_safety_20260808
..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\vision\train_yolo_gpu.py `
  --device 0 --epochs 60 --imgsz 640 --batch 4 --workers 0 --force
```

输出：

- `runtime/yolo_train/xiaou_objects_dataset/`：修正路径并分层拆分后的数据集；
- `runtime/yolo_train/xiaou_objects_gpu/weights/best.pt`：GPU 训练权重；
- `models/xiaou_objects_gpu.onnx` 与 `.names`：树莓派兼容模型；
- `runtime/yolo_train/xiaou_objects_gpu/gpu_training_report.json`：GPU 与验证指标。

## 相机预览

桌面 GPU 直接使用 `.pt`：

```powershell
..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\02_yolo_detect.py `
  --model runtime/yolo_train/xiaou_objects_gpu/weights/best.pt `
  --device cuda:0
```

树莓派或 CPU 回归使用 ONNX：

```bash
python3 robot_ai/02_yolo_detect.py --model xiaou_objects_gpu.onnx --device cpu
```

离线图片验收（不打开串口、不运动）：

```powershell
..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\vision\validate_yolo_adapters.py `
  --models xiaou_objects_gpu.onnx,xiaou_all.onnx --device cpu
```

## Transformer 仿真训练与回放

```powershell
..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\decision\train_transformer_policy.py `
  --device cuda --episodes 512 --sequence-length 8 --epochs 16 --batch-size 64

..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\decision\replay_transformer_policy.py `
  --device cuda --checkpoint runtime/decision/transformer_policy.pt `
  --scenarios bottle,cup,pen,desktop_item,tissue_pull,cola
```

导出并量化后，`.pt` 和 `.onnx` 使用同一个回放入口；`.onnx` 自动走
ONNX Runtime CPU，不会被当成 PyTorch 权重读取：

```powershell
..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\decision\quantize_transformer_policy.py `
  --checkpoint runtime/decision/transformer_policy_deep.pt `
  --output runtime/decision/transformer_policy_deep.onnx `
  --output-int8 runtime/decision/transformer_policy_deep.int8.onnx

..\..\.venv_transformer_gpu\Scripts\python.exe robot_ai\decision\replay_transformer_policy.py `
  --checkpoint runtime/decision/transformer_policy_deep.int8.onnx `
  --device cpu --scenarios bottle,cup,pen,desktop_item,tissue_pull,cola
```

Transformer 只输出任务策略、阶段和有界候选参数；动作范围、碰撞、反馈和硬件状态仍由后续门禁决定。当前训练标签来自可解释仿真规则，不是真实抓取成功率。
