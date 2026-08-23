# 六轴抓取仿真审查（2026-08-07）

本次只运行离线数值仿真和静态检查，未打开 SocketCAN、未连接树莓派、未发送真实关节命令。所有报告中的 `real_can_opened=false` 与 `real_motion_enabled=false` 均为硬门禁结果。

## 机械模型复核

- POE 与 URDF 关节轴最大误差：`3.97e-9`。
- POE 与 URDF home 位姿最大误差：`1.00e-12`。
- visual mesh 8 个、collision mesh 8 个，模型文件完整。
- 这只证明 CAD/POE/URDF 数学一致，不代表编码器零位、方向、限位和真实 TCP 已标定。

## 世界模型复核

当前九点单应性输出范围约为：

- X：`254.8..280.3 mm`
- Y：`-239.1..-219.4 mm`

运行时工作区门禁仍是 X `80..360 mm`、Y `-180..180 mm`。因此带生产门禁的 20 次仿真为 `20/20` 拒绝，原因全部是 `outside_robot_workspace`。这不是允许直接扩大工作区的依据，必须先实测并重新确认 `base_link` 原点、正方向、桌面坐标和九点标定点。

## 单次抓取

像素 `(318,297)`、假设抓取高度 `0.03 m`：

- 单应性输出：`(268.50, -230.27) mm`。
- 当前节点姿态 `RPY(pi,0,yaw)`：approach、descend、lift IK 失败，完整路径失败。
- CAD TCP 对齐姿态：五段全部收敛，位置残差约为微米到几十微米量级。
- 该 CAD 姿态仅用于诊断，未写入生产 ROS2 参数。

## 20 次蒙特卡洛

噪声模型：5 帧检测、像素 sigma `1.5 px`、单应性残差 sigma `0.83 mm`、物体 XY sigma `2 mm`、yaw sigma `3 deg`、抓取高度 sigma `3 mm`。

### 生产门禁开启

报告：`runtime/simulations/real_world_batch_20260807_enforced_20.json`

- 20/20 在工作区门禁处拒绝。
- 没有目标进入 IK，因此不能用该报告宣称运动规划成功。

### 仅用于诊断，绕过工作区门禁

报告：`runtime/simulations/real_world_batch_20260807_multistart_continuity_20.json`

- 进入 IK 的样本：18/20（另 2 个在校准凸包边缘外）。
- 当前 `RPY(pi,0,yaw)`：0/18。
- CAD TCP 对齐姿态，单初值：17/18。
- CAD TCP 对齐姿态，5 初值 + 连续性选支：18/18。

多初值优化解决的是局部 IK 分支问题；它没有放宽关节位置、速度、加速度限制，也没有修复坐标系或 TCP 姿态错误。

## 已做的代码优化

1. `robot_ai/arm_control/kinematics.py` 增加 `ik_space_multistart`：只在有限候选初值中搜索，优先收敛解，再优先选择距离上一段关节状态最近的分支。
2. `tools/simulate_six_axis_pick.py` 增加 `--orientation-mode current_rpy|cad_tcp`、`--ik-restarts 1|3|5`、`--max-ik-iterations`，并记录每段迭代次数和残差。
3. `tools/simulate_real_world_batch.py` 支持同样的 IK 参数，并输出失败阶段、迭代统计和工作区门禁统计。
4. 增加多初值回归测试；本地 `unittest`：`25 tests, OK`。

## 仍必须优化的地方

1. **先校准 TCP 姿态**：实测工具坐标系后，将 `RPY(pi,0,yaw)` 替换为实际抓取姿态；在此之前不能把 CAD TCP 姿态直接用于实机。
2. **重新做相机到 base 的标定**：确认坐标轴和九点物理位置后，再更新单应性和工作区门禁；不要为了让仿真通过而扩大 Y 范围。
3. **补齐每类物品抓取高度**：`robot_ai/arm_control/config/object_grasp_profiles.json` 当前全部为 `null`，保持锁定是正确行为，必须逐类实测。
4. **接入 MoveIt PlanningScene review-only 检查**：当前数值仿真未替代桌面碰撞、自碰撞、抓取物碰撞和路径碰撞检查。
5. **实测关节参数后再解除硬件门禁**：ID、零位、方向、正负限位、速度/加速度、反馈和急停必须单轴低速小角度验证；当前配置仍不可真实运动。

## 复现命令

```powershell
python tools\simulate_six_axis_pick.py --orientation-mode current_rpy --max-ik-iterations 150 --output runtime\simulations\six_axis_pick_20260807_current_rpy.json
python tools\simulate_six_axis_pick.py --orientation-mode cad_tcp --ik-restarts 5 --max-ik-iterations 150 --output runtime\simulations\six_axis_pick_20260807_cad_tcp.json
python tools\simulate_real_world_batch.py --trials 20 --seed 20260807 --max-ik-iterations 150 --ik-restarts 5 --ignore-workspace-gate --output runtime\simulations\real_world_batch_20260807_multistart_continuity_20.json
python -m unittest discover -s tests -q
```

结论：机械几何模型当前自洽；抓取链路尚未达到实机可用。下一步应优先完成 TCP 姿态和相机坐标系实测，再进行 MoveIt 碰撞审查，最后才是单轴实机标定和逐步解除运动锁。
