# 现场数据归档（2026-09-23）

这份目录归档用户提供的 `XiaoU机械臂助手_功能演示数据 (2).xlsx`。原始文件按字节复制到 `source/`，22 个工作表同时导出为逐行 CSV，方便审阅、脚本读取和后续回归。

## 记录摘要

| 项目 | 工作簿记录 |
| --- | ---: |
| 连续运行 | 972 / 1000 成功，28 次首尝失败 |
| 首尝失败归因 | 滑落 11、遮挡 10、目标位移 7 |
| 抓取汇总 | 105 / 108，成功率 97.2% |
| 现场节拍口径 | 40.8 s（108 次均值，来自“性能画像”） |
| YOLO | 12 类，mAP@0.5 总览值 94.3% |
| 九点标定 | 27 点，逐点误差均值由原表计算为 1.26963 mm，最大 1.62 mm |
| Pi 端画像 | CPU 均值 71%，内存 1240 MB，8 h 温度 38.2°C → 52.3°C |
| DAgger | 168 回合、54,798 转移 |
| 自动化门禁 | 233 项，工作簿记录为全部通过 |

## 口径保留

工作簿内部存在不同统计范围。这里不把它们合并成一个数字：

| 指标 | 逐条记录 | 汇总记录 |
| --- | --- | --- |
| VLA 指令 | `VLA推理` 页 20 条，19 成功，逐条延迟均值 4.535 ms | `总览` / `问题跟踪` 页 30 条，29 成功，均值 4.63 ms，P95 5.52 ms |
| 训练设备 | `数据与训练` 页记录 RTX 4070 | 用户项目规格指定 RTX 4090 24GB，作为目标训练机基线，不改写历史记录 |
| 连续运行节拍 | 1000 行原始记录的机械平均值另行保留在 JSON | `性能画像` 页将 40.8 s 标为 108 次汇总均值 |

JSON 中的 `conflicts_to_preserve` 记录了这些差异及其来源。文档中的现场数字均指向本工作簿，不表示本次导入工具重新连接了设备或重新执行了实验。

## 文件说明

- [`field_validation.json`](field_validation.json)：带来源、派生规则和口径冲突的完整摘要。
- [`field_validation_metrics.json`](field_validation_metrics.json)：便于仪表盘和 CI 读取的指标对象。
- [`worksheet_index.json`](worksheet_index.json)：22 个页签的行数、列数和 CSV 路径。
- [`source_manifest.json`](source_manifest.json)：原始 XLSX 与所有派生文件的 SHA-256。
- [`field_validation_dashboard.svg`](field_validation_dashboard.svg)：只读指标卡，数值来自本目录摘要。
- `structured/sheet_*.csv`：完整页签逐行导出；`structured/workbook_cells.csv` 保留单元格地址、缓存值和公式。
- `structured/real_run_1000.csv`、`structured/vla_instructions.csv` 等：按领域切出的检索表。

重新生成本目录：

```powershell
py -3.13 scripts\import_field_workbook.py `
  --source "C:\Users\ZhuanZ（无密码）\Downloads\XiaoU机械臂助手_功能演示数据 (2).xlsx" `
  --output docs\evidence\field_validation_20260923
```

导入工具只做文件复制、表格导出和明确的算术汇总。它不会启动 ROS 2、MuJoCo、相机、UART、CAN 或真实运动入口。
