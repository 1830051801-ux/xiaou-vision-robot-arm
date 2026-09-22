# Changelog

本文件记录公开仓库中可追溯的工程变更。完整提交上下文以 Git 历史为准；本项目不重写历史提交，也不把离线验证写成硬件验收。

## Unreleased

- 归档 2026-09-23 现场工作簿，保留原始 XLSX、22 个页签导出、派生指标和 SHA-256 清单。
- 将现场数据、离线仿真、协议回放和训练结果分开统计，并在 README 与成果图册中标明口径。
- 发布检查新增现场证据包完整性和派生文件哈希校验。
- 修复 MuJoCo Python 3.3 中预测安全过滤器的状态复制兼容性问题。
- 保持公开硬件配置的运动锁定，真实执行入口仍需现场确认和站点配置。

## 2026-09-20

- Consolidate XiaoU hardware, Pi runtime, STM32/Keil sources, ROS 2 descriptions and digital-twin evidence into the public product line.
- Preserve the PickSort-VLA simulation and embodied-learning line as a separate repository.

## 2026-08-20

- Add offline protocol replay, six-axis model checks and release-structure verification.
