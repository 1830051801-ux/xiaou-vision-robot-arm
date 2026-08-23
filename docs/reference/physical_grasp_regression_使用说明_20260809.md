# 真实几何模型离线抓取回归

`tools/simulate_physical_grasp.py` 在桌面 GPU 环境运行 MuJoCo DIRECT 回归。它从
`ros2_ws/src/xiaou_arm_description/urdf/xiaou_arm_display.urdf.xacro` 解析六个关节，
加载同目录下的 8 个 collision STL，并用 `robot_geometry_renders/world_model/robot_world_model.urdf`
校验 CAD 世界网格引用。桌面、瓶子、水杯、笔、桌面物品、纸巾堆和可乐由 MuJoCo 世界中的刚体几何组成。

## 运行

```powershell
& D:\机械臂\.venv_transformer_gpu\Scripts\python.exe tools\simulate_physical_grasp.py `
  --trials-per-scenario 8 `
  --max-sampling-attempts 200 `
  --output runtime\simulations\physical_grasp_regression_20260809.json
```

脚本会在 `D:\xiaou_mujoco_runtime` 暂存 ASCII 路径下的 collision 网格，避开 MuJoCo Windows
加载器对中文路径的限制；源文件字节不修改。报告会记录每个随机候选因 IK、桌面碰撞、自碰撞或
抓取接触门被拒绝的次数，再记录最终通过的 trial。

## 边界

- 机器人轨迹采用 MuJoCo 碰撞场景中的精确 POE/五次多项式回放；物体的自由刚体、重力、接触和释放仍由 MuJoCo 推进。
- 关节限位、速度/加速度、链节密度和抓取高度是临时离线假设，不是编码器或 F407 实测值。
- 静态 CAD 夹爪保留，并增加固定 jaw contact proxy；真实夹爪开合、力反馈和软组织/纸巾形变尚未标定。
- CAD world STL 是大型 ASCII 导出，当前作为源引用审计；实际碰撞世界使用桌面和类别参数化刚体，避免把未转换的 ASCII 网格冒充已加载。
- `hardware_motion=false`、`serial_opened=false`、`can_opened=false` 固定写入报告；脚本不会启动 ROS 硬件、串口或 CAN。
