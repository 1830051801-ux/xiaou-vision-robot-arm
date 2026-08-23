# 模型、量化与数据接口

## 随仓库发布的推理资产

| 资产 | 用途 | 位置 |
| --- | --- | --- |
| YOLO 高召回配置 | 笔、瓶、可乐、耳机的候选检测 | `raspberry_pi/models/xiaou_objects_gpu_deep.onnx` |
| YOLO 高精度候选 | 需要更保守筛选时的固定候选模型 | `raspberry_pi/runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.onnx` |
| INT8 Transformer | CPU 端策略回放与部署包验证 | `raspberry_pi/runtime/decision/transformer_policy_safety_final_20260813.int8.onnx` |

模型选择由 `robot_ai/vision/config/yolo_model_registry.json` 管理。配置同时记录 SHA-256、类别模式和置信度；加载时哈希或类别模式不符合会失败，而不是静默切换模型。

## 训练与量化

桌面训练依赖在 `requirements_desktop_sim.txt` 中单独列出，避免把 PyTorch、MuJoCo 和训练数据放入 2 GB 树莓派运行环境。相关入口包括：

- `robot_ai/vision/train_yolo_gpu.py`
- `robot_ai/decision/train_transformer_policy.py`
- `robot_ai/decision/quantize_transformer_policy.py`
- `tools/finetune_yolo_candidates.py`
- `tools/evaluate_transformer_holdout.py`

训练产生的 `.pt`、数据集、运行目录和候选权重默认被忽略。要把新模型纳入发布，请同时更新模型登记、类别 schema、哈希、离线测试和部署包检查，不要直接覆盖现有生产配置。

## 新图片数据接入

建议每批新增数据都保留：

```text
image
label
class schema revision
camera resolution and acquisition condition
train/validation split seed
model hash
evaluation report
```

模型文件和数据目录不是同一回事：公开发布的仓库保留可运行的模型与接口，原始数据集应按照授权、隐私和体积单独管理。
