# YOLO 模型注册、安全 Transformer 与整链压力验证报告

日期：2026-08-09  
工程：`D:\机械臂\work\raspi_robot_ai_safety_20260808`

## 结论

用户批准的四项工作均已完成：

1. 裸 ONNX 类别元数据回退已经修复。
2. 高召回/高精度 YOLO 模型注册、哈希校验和显式切换接口已经建立。
3. Transformer 已加入危险负样本并重新训练，独立盲测危险误接受从 1762 降到 0。
4. YOLO → 标定 → 46 维观测 → INT8 Transformer → ROS2/硬件安全门已完成 800 次离线压力回归，危险放行 0。

全程 `motion_enabled=false`；未打开真实相机、串口、CAN，未启动 ROS2/MoveIt 或发送运动。

## 1. ONNX 类别元数据修复

原问题：当前 OpenCV 适配器只读取 `.names` sidecar。直接使用 Ultralytics 导出的裸 `best.onnx` 时，即使 ONNX 内部已有四类元数据，仍会回退到 COCO 80 类。

修复：

- sidecar 仍为第一优先级。
- sidecar 缺失时，安全解析 ONNX `names` 元数据。
- 桌面优先使用 `onnx` 读取 protobuf；Pi 环境使用已有 `onnxruntime` 回退。
- 元数据只接受连续 ID、非空且唯一的类别名；使用 `ast.literal_eval/json`，不执行字符串。

实测裸模型：

- 模型：`runtime/yolo_train/xiaou_objects_gpu_deep/weights/best.onnx`
- `.names` 存在：false。
- 解析类别：`pen,bottle,cola,earphone`。
- 新增专项测试：3/3 通过。

## 2. 双 YOLO 模型注册与显式切换

注册表：`robot_ai/vision/config/yolo_model_registry.json`

| profile | 角色 | SHA-256 |
|---|---|---|
| `high_recall` | 优先避免漏检 | `74794520a1fcf6d49738fb420fc8e8d2739b9db9b43f34e47f3e0e9773200d5d` |
| `high_precision` | 优先减少误抓候选 | `d850c87dc58d6b258fc1d0c9684f8fe13bf7351e12f088e7e30f6c90f39595ca` |

安全规则：

- 类别表固定为 v1：`pen,bottle,cola,earphone`。
- 模型文件、类别 schema 和 SHA-256 必须全部匹配。
- 未知 profile、模型缺失、哈希变化、类别漂移均失败关闭。
- `automatic_switching=false`；选择模型不启用运动，也不覆盖生产模型。

接口：

- Python：`UnifiedYolo.from_profile("high_recall")` 或 `high_precision`。
- 桌面预览：`robot_ai/02_yolo_detect.py --profile high_precision`。
- ROS2 perception 参数：`yolo_profile:=high_precision`。
- `yolo_model` 与 `yolo_profile` 互斥，避免两个来源冲突。

注册表实图烟测：

- high_recall：测试图 4 个检测。
- high_precision：测试图 2 个检测。
- 自动切换保持 false；注册表整体通过。

## 3. 安全 Transformer 训练

在原 46 维观测接口上加入五类危险负样本，不改变 ROS2 或 ONNX 输入维度：

- 低检测置信度；
- 未知类别 one-hot；
- 碰撞风险；
- 任一关节离线；
- 目标超出策略工作区。

危险样本目标统一为 `reject/abort`、高风险；动作损失只作用于合法正样本，避免无意义的拒绝动作参数污染抓取回归。

训练了两个候选：每个 8192 条轨迹、72 epoch、约 30% 危险样本、CUDA。两个候选训练验证策略/阶段准确率均为 1.0，选中 seed 20260832。

6000 条全新混合盲测，危险样本 1762 条：

| 模型 | 危险拒绝率 | 危险误接受 | 正样本策略准确率 | 正样本动作 MAE | 风险 MAE |
|---|---:|---:|---:|---:|---:|
| 旧增强模型 seed 20260822 | 0.0 | 1762 | 1.0 | 0.003641 | 0.246533 |
| 安全候选 seed 20260831 | 1.0 | 0 | 1.0 | 0.003886 | 0.006664 |
| 安全候选 seed 20260832 | 1.0 | 0 | 1.0 | 0.003405 | 0.005102 |

独立纯正样本盲测 4096 条：

- 旧增强模型动作 MAE 0.0036166。
- 新安全模型动作 MAE 0.0033724。
- 策略与阶段仍为 1.0；拒绝能力提升没有牺牲正样本动作精度。

最佳候选：

- PyTorch：`runtime/decision/transformer_policy_safety_seed20260832.pt`
- INT8：`runtime/decision/transformer_policy_safety_best_20260809.int8.onnx`
- INT8 大小：700,065 B。
- FP32/INT8 六场景策略、阶段和状态 6/6 一致。

策略输出 `reject` 时现在明确返回 `rejected_by_policy`，不再把拒绝错误标成普通 `candidate`。

## 4. 确定性安全门扩展

神经网络拒绝不是唯一保护。SafetyGate 新增独立检查：

- `detection_fresh`；
- `calibration_valid`；
- 最低检测置信度；
- 六个关节全部在线。

真实模式还必须继续满足原有的：

- `motion_enabled`；
- `hardware_ready`；
- `collision_free`；
- `feedback_verified`。

ROS2 observation bridge 已携带检测新鲜度、标定有效性和来源状态。ROS2 decision node 在非仿真模式强制启用全部新增门。

## 5. YOLO 到安全门的 800 次整链压力测试

合成消息压力集包含 8 类，每类 100 次：合法、低置信度、未知类别、碰撞、关节离线、超工作区、检测过期、标定无效。

结果：

- 合法候选：100/100 通过仿真决策。
- 异常候选：700 个。
- Transformer 主动拒绝：低置信度 100、碰撞 100、关节离线 100。
- 安全门阻断：低置信度、碰撞、关节离线、检测过期、标定无效各 100。
- 前置检查阻断：未知类别 100、超工作区 100。
- 危险放行：0。
- 真实硬件候选：0。

证据：`runtime/simulations/perception_decision_safety_stress_20260809.json`

## 6. 真实图片链发现的当前阻断

两套模型都在现有 20 张验证图片上实际运行：

| profile | 检测数 | 落入标定区域 | 发布位姿 |
|---|---:|---:|---:|
| high_recall | 68 | 0 | 0 |
| high_precision | 60 | 0 | 0 |

当前九点标定像素范围只有：

- `u=303..336 px`
- `v=285..316 px`

现有验证图片的检测中心没有落入这个小区域，因此真实图片链在 `outside_calibrated_image_region` 处停止。并且 `object_grasp_profiles.json` 中各类别抓取高度仍为 null；即使检测进入标定区域，也会先被 `configuration_incomplete` 阻断。

所以必须区分：

- 实际 YOLO 推理和标定区域检查已经执行；没有位姿候选。
- 下游 800 次通过的是合成检测消息压力测试。
- 不能把它描述为真实相机闭环或真实抓取验证。

硬件与相机回来后，应重新做覆盖实际桌面工作区的相机标定，并测量每类抓取高度、夹爪开合和放置位姿。

## 7. 树莓派双模型安全候选包

候选包：`runtime/deployment/xiaou_pi_dual_model_safety_candidate_20260809.tar.gz`

- profile set：`dual-candidate`。
- 39 个文件，21,928,193 B。
- SHA-256：`7442dbbe7e43d47b2f0ce44b34cdfa3a96f15866f4e72a5f9764610e0835ade0`。
- 包含 high_recall、高精度 YOLO、模型注册表和安全 Transformer INT8。
- 解包烟测：两套 YOLO 哈希/类别/推理通过，安全 Transformer 元数据通过。
- 运行策略：一次只加载一个 YOLO profile，Transformer 保持 INT8 CPU；静态内存预算仍为 1280/2048 MB。
- 这是候选包，未 SSH、未上传、未替换现有 Pi 版本；RSS/FPS 仍须上机测量。

## 8. 最终回归

- 单元测试：57/57 通过。
- 全工程 compileall：通过。
- ROS2 架构审计：通过，失败 0。
- 安全 Transformer ROS2 离线回放：通过，失败 0。
- `motion_enabled=false`、`ros2_process_started=false`、`hardware_motion=false`。

当前推荐：离线决策后续使用安全 Transformer seed 20260832；YOLO 保留显式 high_recall/high_precision 双 profile。下一项最高优先级不再是继续堆训练，而是等待新图片扩大类别与场景覆盖，并在相机回来后重做实际工作区标定。
