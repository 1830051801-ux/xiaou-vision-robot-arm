# 成果归档与仓库边界

本仓库是小U六轴桌面机械臂的公开工程入口。2026-09-20 的整理将原有 `embodied-arm-learning` 的数字孪生、规划和离线评测源码纳入 `research/embodied-arm-learning/`，使机械、控制、感知、ROS 2 与离线研究成果从同一入口可追溯。

## 内容地图

| 方向 | 公开入口 | 可复现边界 |
| --- | --- | --- |
| 机械与控制 | `stm32_keil/`、`raspberry_pi/robot_ai/arm_control/` | 六轴模型、Pi-F407 UART、F407-CAN 与 Keil 工程；公开配置默认运动锁定 |
| 结构模型 | `docs/visuals/mechanical_model/`、`raspberry_pi/ros2_ws/src/xiaou_arm_description/` | 由本地模型导出的渲染和 ROS 2 网格；渲染不替代尺寸实测或干涉验收 |
| 感知与交互 | `raspberry_pi/robot_ai/vision/`、`raspberry_pi/robot_ai/decision/` | YOLO/ONNX 注册、坐标处理和 Transformer 回放接口；不把离线回放写成实机闭环 |
| 数字孪生 | `research/embodied-arm-learning/` | 过程图 Action-Chunk Transformer、扩散候选、约束投影、ROS 2 工件和离线评测 |
| 离线证据 | `docs/evidence/offline_validation_20260820/`、`docs/evidence/consolidation_20260922/` | 单元测试、协议回放、模型一致性和数值取放；没有开启相机、串口、CAN 或真实机械臂 |
| 现场结果 | `docs/evidence/field_validation_20260923/` | 用户提供的现场工作簿、真实运动闭环、桌面抓取、桌面整理和垃圾清理记录 |

## 来源与保留方式

| 原仓库 | 固化提交 | 归档位置 | 保留原因 |
| --- | --- | --- | --- |
| `1830051801-ux/embodied-arm-learning` | `034c1c31098b5d04580abcb6f5cf730c8b8c715d` | `research/embodied-arm-learning/` | 保留独立数字孪生、评测资源和原始说明 |
| 本地机械模型工作区 | 2026-08 导出 | `docs/visuals/mechanical_model/` 与 Release 附件 | 轻量渲染随源码；原始 STEP 与去重网格以压缩包分发 |

每个导入目录都含 `UPSTREAM.md`。它记录来源、固化提交、许可证处理和公开范围。机器可读清单位于 [CONSOLIDATION_MANIFEST.json](CONSOLIDATION_MANIFEST.json)。

## 使用顺序

1. 先运行根目录的 `scripts/verify_release.py` 和 `raspberry_pi` 的离线测试；
2. 用 `docs/ROS2.md` 浏览模型与规划预览；
3. 需要研究数字孪生时，进入 `research/embodied-arm-learning/`，按其 README 的离线流程运行；
4. 只有在现场逐项确认硬件前置条件后，才讨论任何真实执行入口。

2026-09-22 的增量仿真证据见 [`evidence/consolidation_20260922/`](evidence/consolidation_20260922/)。其中同时保留了可收敛的模型 TCP 对照和当前 RPY 姿态未收敛的失败结果，便于后续标定，而不是把诊断结果写成实机能力。

2026-09-23 的现场工作簿归档见 [`evidence/field_validation_20260923/`](evidence/field_validation_20260923/)。其中保留原始 XLSX、22 个页签的逐行导出、派生指标和来源哈希；仿真回归与现场运行数据按不同目录分别统计。

这次整理不删除历史仓库，也不改变原始提交中的证据含义。旧仓库会保留迁移说明，以便已有链接继续可访问。
