# Transformer 与 YOLO 增强训练及数据接口报告

日期：2026-08-09  
安全边界：全程离线，`motion_enabled=false`，未打开相机、串口、CAN、真实 ROS2/MoveIt 或机械臂运动。

## 结论

Transformer 得到一个明显优于上一轮的增强候选；YOLO 没有出现所有标准指标同时提升的单一替代品，但得到一个误检更少、光照/模糊条件更稳定的“高精度候选”。当前生产模型没有被覆盖，后续可以按任务在“高召回”和“高精度”之间选择。新增图片的数据接口和六类扩展规划已经建立。

## Transformer 增强结果

训练了三个独立候选：

- 每个候选 6144 条合成轨迹，训练 4915、验证 1229。
- 序列长度 8、batch 128、64 epoch、CUDA。
- 随机种子：20260821、20260822、20260823。
- 三个候选的策略与阶段验证准确率均为 1.0。
- 验证动作 MAE 分别为 0.00358810、0.00354472、0.00370553。

随后使用完全不同的种子 20260901 生成 4096 条盲测场景，把旧模型和三个候选放在同一数据上比较：

| 模型 | 策略准确率 | 阶段准确率 | 动作 MAE | 最大动作误差 |
|---|---:|---:|---:|---:|
| 上一轮模型 | 1.0 | 1.0 | 0.0173286 | 0.167015 |
| seed 20260821 | 1.0 | 1.0 | 0.00363974 | 0.057175 |
| seed 20260822 | 1.0 | 1.0 | 0.00363616 | 0.044567 |
| seed 20260823 | 1.0 | 1.0 | 0.00373620 | 0.047639 |

选中种子 20260822：盲测动作 MAE 相对旧模型下降约 79.0%，最大动作误差下降约 73.3%。

最佳候选：

- PyTorch：`runtime/decision/transformer_policy_improved_seed20260822.pt`
- FP32 ONNX：`runtime/decision/transformer_policy_improved_best_20260809.onnx`
- INT8 ONNX：`runtime/decision/transformer_policy_improved_best_20260809.int8.onnx`
- INT8 大小：700,064 B。
- FP32/INT8 六场景策略、阶段和状态全部 6/6 一致。
- 量化最大动作差 0.00595771、最大风险差 0.00083904。
- ROS2 离线回放通过，6/6 个硬件候选仍被安全门阻断。

限制：训练和盲测仍来自同一套合成规则，只能证明规则拟合和接口精度提高，不能等价为真实抓取率提高。

## YOLO 三条增量训练路线

所有候选都从当前 deep 最佳权重开始，使用 RTX 3050、640 输入、batch 4、AdamW 和低学习率微调。

| 模型 | Precision | Recall | mAP50 | mAP50-95 | 判断 |
|---|---:|---:|---:|---:|---|
| 当前 deep 基线 | 0.90563 | 0.97500 | 0.97042 | 0.54292 | 高召回生产候选 |
| balanced 40 epoch | 0.95418 | 0.95833 | 0.96274 | 0.54789 | 高精度候选 |
| robust 40 epoch | 0.91707 | 0.97240 | 0.96627 | 0.53950 | 不推荐 |
| polish 30 epoch | 0.88537 | 0.97500 | 0.96554 | 0.54632 | 不推荐 |

balanced 的 Precision 和 mAP50-95 提高，但 Recall 与 mAP50 小幅下降，因此“全指标无回退”自动升级门拒绝覆盖生产模型。这是预期保护行为。

## 光照、模糊与 JPEG 回归

使用 20 张验证图片，分别生成原图、变暗、变亮、高对比、5×5 模糊和 JPEG Q40，共 120 个输入；通过树莓派兼容的 OpenCV ONNX 路径，在 conf=0.10、IoU=0.5 下做类别感知匹配：

| 模型 | TP | FP | FN | Precision | Recall | F1 |
|---|---:|---:|---:|---:|---:|---:|
| deep 基线 | 331 | 82 | 23 | 0.80145 | 0.93503 | 0.86310 |
| balanced 高精度候选 | 325 | 40 | 29 | 0.89041 | 0.91808 | 0.90403 |

高精度候选把误检从 82 降到 40，聚合 F1 提高约 0.041；代价是漏检增加 6 个。对于“误抓的代价高于偶尔不抓”的桌面操作，它更有价值。

候选文件：

- `runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.pt`
- `runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.onnx`
- `runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.names`

同轮 CPU 公平基准，单线程、预热 10 次、90 个样本：

| 模型 | mean | p50 | p95 |
|---|---:|---:|---:|
| deep 基线 | 165.93 ms | 163.71 ms | 179.52 ms |
| 高精度候选 | 162.23 ms | 160.37 ms | 172.81 ms |

两者结构和大小相同，速度没有实质退化。当前数据仍只有 `pen/bottle/cola/earphone`，没有真实 `cup/tissue`，继续训练旧图片无法补出这两类能力。

## 后续图片数据接口

已新增：

- v1 稳定类别表：`robot_ai/vision/config/yolo_class_schema_v1.json`
- v2 六类草案：`robot_ai/vision/config/yolo_class_schema_v2_draft.json`
- 只读批次审计：`tools/inspect_yolo_dataset_intake.py`
- 完整接入规划：`docs/YOLO新增图片数据集接入与增量训练规划_20260809.md`

现有 deep 数据集接口烟测：100 张图、100 个标签、缺失 0、错误 0、重复组 0，`training_ready=true`。

后续收到新图片时先运行：

```powershell
& D:\机械臂\.venv_transformer_gpu\Scripts\python.exe tools/inspect_yolo_dataset_intake.py `
  --source "D:\待接入数据\batch_001" `
  --schema robot_ai/vision/config/yolo_class_schema_v2_draft.json `
  --batch-name batch_001 `
  --mode unlabeled `
  --compare-images my_all_data/images,runtime/yolo_train/xiaou_objects_dataset_deep/images `
  --output runtime/yolo_intake/batch_001_unlabeled.json
```

如果只增加现有四类，执行保留旧数据的增量微调；如果出现 `cup/tissue`，使用 v2 六类检测头和旧四类 + 新两类的合并数据全量训练。类别 ID 不允许临时改写。

## 新增验证工具

- `tools/evaluate_transformer_holdout.py`：独立种子批量比较 Transformer 检查点。
- `tools/finetune_yolo_candidates.py`：同基线多增强路线微调、验证、回退门与候选导出。
- `tools/evaluate_yolo_photometric_robustness.py`：OpenCV/ONNX 光照、对比度、模糊、JPEG 回归。
- `tools/inspect_yolo_dataset_intake.py`：新数据图片、标签、类别、重复和训练就绪度审计。

## 最终安全回归

- 全套单元测试：46/46 通过。
- `compileall`：`robot_ai/tests/tools/ros2_ws/src` 全部通过。
- 增强 INT8 ROS2 离线回放：6/6 场景通过，6/6 被真实运动安全门阻断。
- 最终配置：`motion_enabled=false`、`hardware_motion=false`、`ros2_process_started=false`。

当前建议：保留 deep 作为高召回模型；新增 balanced 作为高精度候选；Transformer 使用 seed 20260822 增强候选继续离线验证。等新图片到位后，再根据真实类别和采集条件重新决定 YOLO 主模型。
