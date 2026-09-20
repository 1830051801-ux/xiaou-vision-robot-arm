# 结构与仿真可视化

| 资源 | 用途 | 说明 |
| --- | --- | --- |
| `mechanical_model/robot_full_iso.png` | 六轴整机结构总览 | 本地机械模型导出；用于结构展示，不代表实测尺寸公差 |
| `mechanical_model/gripper_iso.png` | 末端执行器等轴视图 | 方便检查夹爪主体、法兰与关节连接关系 |
| `mechanical_model/gripper_axis.png` | 末端法兰/夹爪轴向视图 | 用于坐标和安装方向讨论，不能替代装配检验 |
| `xiaou-stack.svg` | 从结构到控制与离线验证的关系图 | 所有箭头均表示软件/数据接口，不表示已启用实机执行 |
| `simulation/multitask_factory_cell_dashboard.png` | 数字孪生评测图 | 用于展示离线任务评测结果 |

原始 STEP、STL 与中间网格以 Release 附件分发，源码仓库保留轻量网格、渲染和校验清单。
