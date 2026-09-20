# 本轮整理验证（2026-09-20）

| 执行项 | 结果 |
| --- | --- |
| Pi 单元测试 `python -m unittest discover -s tests -q` | 130 通过 |
| 数字孪生单元测试（导入目录） | 51 通过 |
| `scripts/verify_release.py` | 发布结构、模型、Keil 引用与默认锁检查通过 |
| `simulate_taught_cola_grasp.py` | MuJoCo 3.10.0，14 个阶段，保留模型自碰接触记录 |
| `render_taught_replay.py` | 168 帧 GIF、四阶段预览、TCP 轨迹图与展示采样数据 |

`taught-cola.json` 是本轮新运行的运动学碰撞回放。模型关节位置按轨迹直接给定，不评价真实电机跟踪性能；夹爪没有执行，接触仅记录。所有硬件接口均未打开。

重现命令（从 `raspberry_pi/` 执行，需 NumPy、MuJoCo、Matplotlib、ImageIO、Pillow）：

```bash
python tools/simulate_taught_cola_grasp.py --output ../docs/evidence/consolidation_20260920/taught-cola.json --ascii-asset-root /tmp/xiaou-replay
python tools/render_taught_replay.py --report ../docs/evidence/consolidation_20260920/taught-cola.json --mjcf /tmp/xiaou-replay/cola/xiaou_taught_cola.xml --output ../docs/visuals/simulation
```

Windows 将 `/tmp/xiaou-replay` 换为可写的纯英文路径即可。报告内公开路径已经归一化。
