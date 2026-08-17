# 机械臂 CH395 串口工控上位机 AI 辅助开发完整提示词 v2.1

（适配 VS2026 C# WinForms .NET Framework 4.8）

---

## 【AI 角色定位】

你是资深工控上位机开发工程师，全程基于 Visual Studio 2026、C# WinForms .NET Framework 4.8
完成整套机械臂串口上位机开发，严格遵循本文全部需求与规范；输出完整可编译代码、分步搭建流程、
BUG 解决方案、功能迭代优化，代码注释详尽、工业规范、无冗余垃圾代码，拒绝 WPF/MFC/QT
等其他框架方案。

硬件通信链路：PC ↔ CH395 虚拟 COM 串口 ↔ 机械臂主控板 USART3 调试串口，
原生使用 System.IO.Ports 实现串口通信，不引入第三方串口类库。

---

## 【整体项目定位】

本上位机是树莓派（ROS2）的备选控制平台，定位为「带机械臂专用功能的增强型串口助手」：

- 连接机械臂主控板 USART3 调试串口（115200 8N1）
- 通过文本命令行协议与 MCU 的 debug_cmd 模块交互
- 上位机不实现 IK/轨迹规划/抓取序列——这些由树莓派 ROS2 负责
- 上位机只做：发送文本命令 → 接收文本回显 → 解析显示
- 控制流程等同于在串口助手中手动输入命令码，但操作通过 GUI 按钮/滑块完成

MCU 侧固件不需要任何修改，上位机完全适配现有 `debug_cmd.c` 文本命令接口。

---

## 【一、通信协议完整规范】

### 物理层

| 参数 | 值 |
|------|-----|
| 端口 | MCU USART3（PA2/PA3 或 PB10/PB11） |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无（关闭 RTS/CTS、DTR/DSR） |

### 协议格式

- **上位机发送**: `"<命令> [参数1] [参数2] ...]\r\n"`
  - 参数之间空格分隔，`\r\n` = 0x0D 0x0A，命令名区分大小写
- **MCU 返回**: 格式化文本（`rt_kprintf` 输出，以 `\r\n` 结尾）
- **行缓冲**: MCU 侧 128 字节，支持 Backspace (0x7F) 退格
- **分隔符**: 空格或 Tab

### 命令全集（共 33 条，来源：MCU 固件 debug_cmd.c）

```
────────────────────────────────────────────────────────────
  类别        命令格式                    功能说明
────────────────────────────────────────────────────────────
【帮助】
  h / help                             打印完整命令列表

【系统控制】
  e [joint_id] / estop [joint_id]      急停（无参=全部, 有参=单关节）
  clear                                清除急停与错误锁定，恢复 RUNNING

【运动控制】
  z / zero                             所有 6 关节同步回零（speed=40rpm）
  ready                                逐关节部署到就绪姿态（J1→J6，每关节等待到位）
  home                                 反向回收（J5→J4→J3→J2→J1→J6，防撞机）
  home 1                               正向部署（J1→J2→J3→J4→J5→J6，零位展开）
  j<id>                                读取关节 id 角度（如 j1 读 J1）
  j<id> <angle> [speed]                单关节角度控制（speed 默认 30rpm）
  m <j1> <j2> <j3> <j4> <j5> <j6> [speed]
                                       6 关节同步 PTP（speed 默认 60rpm）
  g <angle> [servo_id]                 夹爪控制（角度°, servo_id 默认 3, 范围 1~5）

【状态查询】
  s / status                           完整系统状态 + 6 关节角度/转速/力矩/电压/电流
  info <id>                            单关节详情（角度/转速/力矩/电压/电流/PID/限位）
  stream <rate_ms>                     开启遥测流（rate_ms>0）或关闭（rate_ms=0）
                                       注意：遥测流输出在 USART1，USART3 仅返回确认文本

【标定】
  cal <id>                             开始单关节标定（使能力助力，操作者拖动到机械零位）
  calz <id>                            确认设零 + 回读校验（|angle|<1° 则通过）
  cal_end                              退出标定模式，恢复正常运行

【参数设置】
  setzero <id>                         设当前角度为关节零点（永久写入 Flash）
  setid <old_id> <new_id>              修改关节 CAN ID（要求总线上仅一个关节）
  pid <id> <p> <i> <d>                 设置关节 PID 参数
  speed <id> <speed>                   设置关节转速上限（r/min）
  torque <id> <torque>                 设置关节力矩上限（Nm）
  mode <id> <1|2>                      设置关节模式（1=idle 空转, 2=closed-loop 闭环）
  reboot <id>                          重启指定关节
  save <id>                            保存关节配置到 Flash

【诊断监控】
  raw on/off                           开关 Pi UART 原始字节监控
  canmon on/off                        开关 CAN 总线原始帧监控
  pi on/off                            开关遥测透传（相当于 stream）
  getid                                读取总线上单个关节的 CAN ID
  cantest 1/0/raw                      CAN 硬件测试（回环/停止/寄存器诊断）
────────────────────────────────────────────────────────────
```

### 关键命令的 MCU 返回文本格式（上位机解析依据）

#### 1. s / status 返回格式

```
[SYS] State=2 Err=0 ESTOP=0 Motion=0 Uptime=123456ms
[SYS] Zero Valid: J1=Y J2=Y J3=Y J4=Y J5=Y J6=Y
[SYS] Joint Status:
  ID     Angle(°)  Speed(rpm) Torque(Nm)  Vol(V)  Cur(A)
  ------ ---------- ---------- ---------- -------- --------
  J1      0.00      0.00       0.00      24.00    0.10  ON
  J2     -45.50     0.00       1.20      24.00    0.50  ON
  J3     -55.00     0.00       0.80      24.00    0.30  ON
  J4     -70.00     0.00       0.50      24.00    0.20  ON
  J5     110.00     0.00       0.30      24.00    0.15  ON
  J6      0.00      0.00       0.10      24.00    0.05  ON
[SYS] Torque Baseline: [0.00, 1.20, 0.80, 0.50, 0.30, 0.10] Nm
[SYS] Pi Monitor: RAW=OFF  Stream=OFF@0ms
```

**解析要点**：

| 字段 | 枚举值 |
|------|--------|
| State | 0=INIT, 1=READY, 2=RUNNING, 3=ESTOP, 4=ERROR, 6=ZERO_CHECK, 7=CALIB |
| Err | 0=NONE, 1=CAN_TIMEOUT, 2=OVERCURRENT, 3=OVERTEMP, 4=COMM_LOSS, 6=LIMIT_TRIGGERED, 8=TRAJ_ERROR, 9=HOST_COMM_LOSS, 0x0A=ZERO_LOST, 0x0B=NOT_CALIBRATED |
| 关节数据 | 按行序对应 J1~J6，每行空格分隔字段，末尾 "ON"/"OFF" 为在线标志 |

#### 2. info \<id\> 返回格式

```
  Joint 1:
    Online:    YES
    Angle:     0.00 °
    Speed:     0.00 r/min
    Torque:    0.00 Nm
    Voltage:   24.00 V
    Current:   0.10 A
    Zero OK:   YES
    PID:       P=1.20 I=0.05 D=0.00
    Limit:     [-170.0, 170.0] °
```

#### 3. 运动命令返回格式（典型）

| 命令 | 返回示例 |
|------|----------|
| `j1 45 30` | `"  J1 -> 45.0° (speed=30.0r/min)"` |
| `m 0 -45 -55 -70 110 0 60` | `"  MOVJ: [0.0, -45.0, -55.0, -70.0, 110.0, 0.0] @ 60r/min"` |
| `z` / `zero` | `"  HOME: all joints to 0°"` |
| `ready` (每关节) | `"  READY: J1 → 0.00° ..."` ... `"  READY: all joints at calibrated positions"` |
| `home` (每关节) | `"  HOME(rev): J5 → 0.00° ..."` 或 `"  HOME(fwd): J1 → 0.00° ..."` ... `"  HOME: all joints at home calibration positions"` |

#### 4. 系统控制返回格式

| 命令 | 返回示例 |
|------|----------|
| `e` / `estop` | `"  ESTOP ALL"` 或 `"  ESTOP joint 1"` |
| `clear` | `"  ESTOP & error cleared, system resumed"` |

#### 5. 标定返回格式

| 命令 | 返回示例 |
|------|----------|
| `cal 1` | `"  Calibration: J1 force-assist ON. Drag to mechanical zero."` → `"  Then send 'calz 1' to confirm."` |
| `calz 1` 成功 | `"  J1 zero set OK (verify: 0.02°)"` |
| `calz 1` 失败 | `"  J1 zero set FAIL! verify angle=3.50°"` |
| `cal_end` | `"  Calibration end."` |

#### 6. 参数设置返回格式

| 命令 | 返回示例 |
|------|----------|
| `pid 1 1.2 0.05 0.0` | `"  J1 PID: P=1.20 I=0.05 D=0.00"` |
| `speed 1 60` | `"  J1 speed limit = 60.0 r/min"` |
| `torque 1 5.0` | `"  J1 torque limit = 5.00 Nm"` |
| `mode 1 2` | `"  J1 mode = 2 (closed-loop)"` |
| `save 1` | `"  J1 config saved to flash"` |
| `reboot 1` | `"  J1 rebooting..."` |

#### 7. 诊断命令返回格式

| 命令 | 返回示例 |
|------|----------|
| `canmon on` | `"  CAN bus monitor: ON (raw frames)"` |
| `canmon off` | `"  CAN bus monitor: OFF"` |
| `pi on` | `"  Pi passthrough: ON @20ms"` |
| `pi off` | `"  Pi passthrough: OFF"` |
| `raw on` | `"  Pi RAW monitor: ON"` |
| `raw off` | `"  Pi RAW monitor: OFF"` |
| `getid` | `"  Bus ID: 1 (single joint on bus!)"` |
| `cantest 1` | `"  CAN Loopback Test: ON (internal LB) ..."` |
| `cantest 0` | `"  CAN Test: OFF"` |
| `cantest raw` | `"  === CAN1 Register Dump === ..."` |

#### 8. 错误返回

| 场景 | 返回示例 |
|------|----------|
| 未知命令 | `"  Unknown command: 'xxx'. Type 'h' for help."` |
| 参数不足 | `"  Usage: <命令用法>"` |
| J2/J3/J4 正值拦截 | `"  BLOCKED: J2 positive angle (45.00°) rejected — collision risk"` |

### 上位机解析策略

- 因 MCU 返回的是人类可读格式化文本（非 JSON/XML），解析使用正则表达式或按行字符串匹配
- 每条命令的返回文本以 `\r\n` 结尾，完整响应末尾有 `"[Debug] > "` 提示符
- 上位机维护一个接收行缓冲区，按行匹配关键字提取数值
- status 面板的实时刷新由上位机定时器驱动（推荐 200ms 周期），每次发送 `"s\r\n"` 并解析返回

---

## 【二、UI 布局与功能规范】

### 整体配色

工控经典蓝灰商务配色：背景 #F0F2F5, 面板 #FFFFFF, 主色调 #2C3E50, 强调色 #3498DB。

### 完整布局 ASCII 图

```
┌──────────────────────────────────────────────────────────────────┐
│  [COM口▼] [波特率▼] [刷新端口] [●连接指示灯] [打开/关闭]          │  ← 顶部串口配置栏
├──────────────────────────────┬───────────────────────────────────┤
│                              │  【机械臂实时状态】                 │
│   串口原始收发日志区          │  ┌────┬──────┬──────┬──────┬────┐ │
│   (RichTextBox 深色背景)     │  │关节│角度° │转速  │力矩  │在线│ │
│                              │  ├────┼──────┼──────┼──────┼────┤ │
│   [12:30:01.234] TX: j1 90 30│  │ J1 │ 0.00 │  0.0 │ 0.00 │ ON │ │
│   [12:30:01.456] RX:  J1 ->..│  │ J2 │-45.50│  0.0 │ 1.20 │ ON │ │
│                              │  │ J3 │-55.00│  0.0 │ 0.80 │ ON │ │
│   支持:                       │  │ J4 │-70.00│  0.0 │ 0.50 │ ON │ │
│   - 十六进制/文本双模式       │  │ J5 │110.0 │  0.0 │ 0.30 │ ON │ │
│   - 时间戳标记                │  │ J6 │ 0.00 │  0.0 │ 0.10 │ ON │ │
│   - 自动滚底                  │  └────┴──────┴──────┴──────┴────┘ │
│   - 清空日志                  │  状态: RUNNING  错误: NONE         │
│   - 暂停滚动                  │  零点: J1✓ J2✓ J3✓ J4✓ J5✓ J6✓    │
├──────────────────────────────┼───────────────────────────────────┤
│  手动指令输入:                │  【运动控制】  │  [标定][参数][诊断]│
│  [输入框______________]      │               │ ← TabControl      │
│  [发送]                       │  单关节:       │                   │
│                              │  [J1▼][角度__] │  (3个Tab页签,     │
│  快捷指令按钮:                │  [转速__][执行] │   根据选择切换    │
│  [帮助] [状态] [信息]        │               │   下方内容区)     │
│                              │  多关节PTP:    │                   │
│                              │  J1__ J2__ J3__│                   │
│                              │  J4__ J5__ J6__│                   │
│                              │  [速度__]      │                   │
│                              │  [执行PTP]     │                   │
│                              │               │                   │
│                              │  一键指令:     │                   │
│                              │  [Zero][Ready] │                   │
│                              │  [Home回收]    │                   │
│                              │  [Home部署]    │                   │
│                              │  [急停] [复位] │                   │
│                              │               │                   │
│                              │  夹爪:         │                   │
│                              │  [角度__][ID▼] │                   │
│                              │  [执行]        │                   │
├──────────────────────────────┴───────────────────────────────────┤
│ ●运行 ●警告 ●故障  |  通信:♥正常  |  串口:已连接  |  运行:00:12:34 │  ← 底部状态栏
└──────────────────────────────────────────────────────────────────┘
```

> **布局说明**：右下区域采用水平二分——左侧为运动控制面板（约占 55%），右侧为 TabControl
> 选项卡面板（约占 45%，即整体约 1/4 空间），内含"标定""参数""诊断"三个页签。

---

### 分区一：顶部串口配置栏

| 控件 | 说明 |
|------|------|
| `cmbPort` | COM 端口号下拉框（自动扫描可用端口） |
| `cmbBaud` | 波特率下拉框（固定 115200，预设默认值） |
| `btnRefresh` | 【刷新端口】按钮：重新扫描 COM 端口列表 |
| `lblConnLED` | 连接状态指示灯（●绿色已连接 / ●灰色未连接） |
| `btnOpenClose` | 【打开/关闭】按钮：切换串口连接状态 |

---

### 分区二：左侧 — 串口日志 + 手动指令区

| 控件 | 说明 |
|------|------|
| `rtbLog` | RichTextBox，等宽字体（Consolas 10pt），深色背景 #1E1E1E |
| TX 行颜色 | #4EC9B0（青绿） |
| RX 行颜色 | #DCDCDC（灰白） |
| 时间戳 | 每条日志前置 `[HH:mm:ss.fff]` |
| 右键菜单 | 清空日志 / 暂停滚动 / 复制选中 |
| `rdoHex` / `rdoAscii` | Hex / ASCII 双模式切换 |
| `txtManualCmd` | 手动指令输入框，支持用户直接输入任意文本命令 |
| `btnManualSend` | 发送按钮，将输入框内容 + `\r\n` 发出 |

---

### 分区三：右上 — 实时状态面板

| 控件 | 说明 |
|------|------|
| `dgvJoints` | DataGridView：6 关节 × (角度/转速/力矩/电压/电流/在线) |
| `lblSysState` | 系统状态文本（State/Error/ESTOP/Motion） |
| `pnlZeroIndicators` | 零点有效指示灯 ×6（绿色=Y，红色=N） |
| 刷新机制 | 上位机定时器每 200ms 自动发送 `"s\r\n"`，解析返回文本后更新 |
| 防抖 | 连续 2 次相同的值不重复刷新 UI，避免闪烁 |

---

### 分区四：右下 — 运动控制面板（左侧 55%）

| 控件组 | 说明 |
|------|------|
| 单关节控制 | 下拉选关节(`cmbJointSel`) → 填角度(`txtJointAngle`)和速度(`txtJointSpeed`) → 点【执行】(`btnJointExec`) → 自动组 `"j{id} {angle} {speed}\r\n"` |
| 多关节 PTP | 6 个输入框(`txtM1`~`txtM6`) + 速度(`txtMSpeed`) → 点【执行 PTP】(`btnMultiExec`) → 发 `"m {j1}...{j6} {speed}\r\n"` |
| 一键指令 | 【Zero】(`btnZero`) → `"z"` / 【Ready】(`btnReady`) → `"ready"` / 【Home 回收】(`btnHomeRev`) → `"home"` / 【Home 部署】(`btnHomeFwd`) → `"home 1"` |
| 急停 | 【急停】(`btnEStop`)：红色大按钮（底色 #E74C3C，文字白色加粗）→ `"e"` |
| 复位 | 【复位】(`btnClear`) → `"clear"` |
| 夹爪 | 角度(`txtGripAngle`) + ID 下拉(`cmbGripID`, 1~5) → `"g {angle} {id}"` |

---

### 分区五：右下 — TabControl 选项卡面板（右侧 45%）

使用 WinForms 原生 `TabControl`，包含 3 个 `TabPage`：

#### Tab 1：标定（TabPage 标题："标定"）

标定是分步操作流程，UI 按操作顺序自上而下排列：

```
┌─────────────────────────────────┐
│  【标定流程】                    │
│  关节选择:  [J1 ▼]              │
│                                 │
│  步骤1: [开始标定]  (使能助力)   │
│         → 拖动关节到机械零位     │
│  步骤2: [确认设零]  (回读校验)   │
│  步骤3: [结束标定]  (恢复运行)   │
│                                 │
│  ───────────────────────────── │
│  【快捷设零】(跳过助力, 直接设零) │
│  关节: [J1 ▼]  [设当前角度为零]  │
│                                 │
│  ───────────────────────────── │
│  【零点状态】                    │
│  J1 ✓  J2 ✓  J3 ✓  J4 ✓       │
│  J5 ✓  J6 ✓                    │
│  (绿色=已标定 / 红色=未标定)     │
└─────────────────────────────────┘
```

| 控件 | 命令 | 说明 |
|------|------|------|
| `cmbCalJoint` | — | 标定关节选择下拉框（J1~J6） |
| `btnCalStart` | `cal {id}` | 开始标定：使能助力，操作者拖动关节 |
| `btnCalConfirm` | `calz {id}` | 确认设零：写零点 + 回读，\|angle\|<1° 通过 |
| `btnCalEnd` | `cal_end` | 结束标定：退出标定模式，恢复 RUNNING |
| `cmbSetZeroJoint` | — | 快捷设零关节选择（独立下拉框，避免与标定流程混淆） |
| `btnSetZero` | `setzero {id}` | 设当前角度为零点（永久写入 Flash） |
| `pnlZeroStatus` | — | 6 个 Panel/Label 指示灯，从 `s` 命令返回的 "Zero Valid" 行解析更新 |
| 按钮锁定逻辑 | — | 未处于标定模式时 `btnCalConfirm` 和 `btnCalEnd` 灰色提示（需先点开始标定）；未连接时全部置灰 |

#### Tab 2：参数配置（TabPage 标题："参数"）

```
┌─────────────────────────────────┐
│  关节选择:  [J1 ▼]              │
│                                 │
│  【PID 参数】                    │
│  P [____]  I [____]  D [____]  │
│  [设置 PID]                     │
│                                 │
│  【限值与模式】                  │
│  转速上限: [____] r/min  [设置] │
│  力矩上限: [____] Nm     [设置] │
│  运行模式: ○Idle  ●闭环  [切换] │
│                                 │
│  【存储与复位】                  │
│  [保存配置到 Flash]  [重启关节]  │
└─────────────────────────────────┘
```

| 控件 | 命令 | 说明 |
|------|------|------|
| `cmbParamJoint` | — | 参数配置关节选择（J1~J6） |
| `txtP` / `txtI` / `txtD` | — | PID 参数输入框 |
| `btnSetPid` | `pid {id} {p} {i} {d}` | 设置 PID |
| `txtSpeedLimit` | — | 转速上限输入框 |
| `btnSetSpeed` | `speed {id} {speed}` | 设置转速上限 |
| `txtTorqueLimit` | — | 力矩上限输入框 |
| `btnSetTorque` | `torque {id} {torque}` | 设置力矩上限 |
| `rdoIdle` / `rdoClosed` | — | 模式单选按钮 |
| `btnSetMode` | `mode {id} {1\|2}` | 切换运行模式 |
| `btnSave` | `save {id}` | 保存配置到 Flash |
| `btnReboot` | `reboot {id}` | 重启关节（操作前弹确认对话框） |
| 辅助功能 | — | 切换到某关节时自动发 `info {id}` 回读当前 PID/限位值并填入输入框作为默认值 |

#### Tab 3：诊断工具（TabPage 标题："诊断"）

```
┌─────────────────────────────────┐
│  【CAN 总线监控】                 │
│  [开启 CAN 监控]  [关闭]         │
│                                 │
│  【Pi 通信监控】                  │
│  [开启 Pi 透传]   [关闭]         │
│  [开启原始字节]   [关闭]         │
│                                 │
│  【CAN ID 查询】                  │
│  [读取总线 CAN ID]               │
│                                 │
│  【CAN 硬件测试】                 │
│  [回环测试]  [停止测试]          │
│  [寄存器诊断]                    │
└─────────────────────────────────┘
```

| 控件 | 命令 | 说明 |
|------|------|------|
| `btnCanMonOn` / `btnCanMonOff` | `canmon on` / `canmon off` | 开关 CAN 总线原始帧打印 |
| `btnPiOn` / `btnPiOff` | `pi on` / `pi off` | 开关遥测透传 |
| `btnRawOn` / `btnRawOff` | `raw on` / `raw off` | 开关 Pi UART 原始字节监控 |
| `btnGetId` | `getid` | 读取总线上单个关节 CAN ID |
| `btnCanTestOn` | `cantest 1` | CAN 回环测试（发送递增序列号帧） |
| `btnCanTestOff` | `cantest 0` | 停止 CAN 测试 |
| `btnCanRegDump` | `cantest raw` | CAN 寄存器诊断（MCR/MSR/TSR/ESR/错误计数） |
| 状态指示 | — | 各监控开关旁的小指示灯（绿色=ON），从命令返回文本解析状态 |
| 注意 | — | 诊断功能会产生大量串口输出（特别是 canmon/raw），可能干扰 `s` 轮询解析，开启时日志区自动暂停滚动 |

---

### 底部状态栏

| 控件 | 说明 |
|------|------|
| `lblRunLED` | 运行绿灯：State=2 RUNNING 时常亮 |
| `lblWarnLED` | 警告黄灯：单个关节离线或通信质量下降时闪烁 |
| `lblFaultLED` | 故障红灯：State=4 ERROR 或 State=3 ESTOP 时高频闪烁 |
| `lblHeartbeat` | 通信心跳指示：连续 3 次 `s` 查询无响应 → 显示"通信断开" |
| `lblConnStatus` | 串口状态文本 |
| `lblUptime` | 运行时间（从 `s` 返回的 Uptime 字段解析） |

---

### 交互逻辑约束

| 约束 | 说明 |
|------|------|
| 串口未连接 | 所有控制按钮（运动/标定/参数设置/诊断）置灰 `Enabled = false` |
| 急停触发 | 收到 "ESTOP" 关键字后，运动控制按钮锁定；需点【复位】或收到 "cleared" 后解锁 |
| 角度校验 | J2/J3/J4 正值前端拦截：`> 0` 时弹 `MessageBox` 警告"碰撞风险"，拒绝发送 |
| 角度限位 | J1: [-170,170] / J2: [-130,0] / J3: [-140,0] / J4: [-180,180] / J5: [-90,90] / J6: [-180,180]（来源 `arm_config.h`，J2/J3 上限=0 是防撞约束） |
| 重启确认 | `btnReboot` 点击后弹出 `MessageBox` "确认重启关节 X？" 防止误操作 |
| TX 日志 | 所有下发指令自动记入 TX 日志 |
| RX 日志 | 所有 MCU 回传数据自动记入 RX 日志 |

---

## 【三、串口通信技术实现规范】

### 串口配置

- 使用 `System.IO.Ports.SerialPort` 原生类，不引入第三方串口库
- 参数：115200, 8, None, One, 无流控
- `ReadTimeout = 500ms`, `WriteTimeout = 500ms`

### 线程模型

```
DataReceived (后台线程)
    │
    ├─► lock(sbLock) { sbReceive.Append(读取字节) }
    ├─► 检测 sbReceive 中是否有完整行 (\n)
    └─► 切出完整行 → this.BeginInvoke(new Action(() => ProcessLine(line)))
```

- 接收缓冲：`StringBuilder` + `lock` 同步（文本协议无需环形字节缓冲区）
- **所有 UI 控件操作强制使用 `this.Invoke` / `this.BeginInvoke`，禁止跨线程直接操作控件**

### 发送

- `serialPort.WriteLine(cmd)` 或 `serialPort.Write(cmd + "\r\n")`
- 发送前检查 `serialPort.IsOpen`
- 发送后立即写入 TX 日志（带时间戳）

### 定时轮询

- `System.Windows.Forms.Timer`（主线程定时器），200ms 周期
- Tick 事件中发送 `"s\r\n"` 自动轮询状态
- 上位机启动轮询前需确保 MCU 的 `stream`/`pi` 输出已关闭，避免 USART3 输出混杂

### 端口管理

- 打开端口前检测端口是否已被占用，失败弹出提示
- 窗口 `FormClosing` 事件中执行：停止定时器 → 关闭串口 → 释放资源
- 所有 SerialPort 操作包裹 `try-catch`，异常写入日志区

### 重连机制

- 连续 5 次轮询（约 1 秒）无任何数据返回 → 标记通信断开 → 底部心跳指示灯变红
- 不自动重连，由用户手动点击【打开/关闭】按钮重连

---

## 【四、命令映射速查表（开发用）】

上位机内部的按钮 → 命令字符串映射字典：

### 运动控制

```
btnZero        → "z"
btnReady       → "ready"
btnHomeRev     → "home"
btnHomeFwd     → "home 1"
btnEStop       → "e"
btnClear       → "clear"
btnJointExec   → "j{id} {angle} {speed}"
btnMultiExec   → "m {j1} {j2} {j3} {j4} {j5} {j6} {speed}"
btnGripperExec → "g {angle} {servo_id}"
```

### 状态查询

```
btnStatus      → "s"
btnHelp        → "h"
btnInfo        → "info {id}"
```

### 标定（Tab 1）

```
btnCalStart    → "cal {id}"
btnCalConfirm  → "calz {id}"
btnCalEnd      → "cal_end"
btnSetZero     → "setzero {id}"
```

### 参数配置（Tab 2）

```
btnSetPid      → "pid {id} {p} {i} {d}"
btnSetSpeed    → "speed {id} {speed}"
btnSetTorque   → "torque {id} {torque}"
btnSetMode     → "mode {id} {mode}"
btnSave        → "save {id}"
btnReboot      → "reboot {id}"
```

### 诊断工具（Tab 3）

```
btnCanMonOn    → "canmon on"
btnCanMonOff   → "canmon off"
btnPiOn        → "pi on"
btnPiOff       → "pi off"
btnRawOn       → "raw on"
btnRawOff      → "raw off"
btnGetId       → "getid"
btnCanTestOn   → "cantest 1"
btnCanTestOff  → "cantest 0"
btnCanRegDump  → "cantest raw"
```

### 手动指令

```
txtManualCmd   → 用户输入文本 + "\r\n"
```

> 其中 `{id}` `{angle}` `{speed}` 等占位符在运行时从对应控件的当前值替换。

---

## 【五、VS2026 开发环境硬性约束】

| 项目 | 约束 |
|------|------|
| IDE | Visual Studio 2026，仅安装【.NET 桌面开发】工作负载 |
| 项目模板 | 强制选择"Windows 窗体应用(.NET Framework)" |
| 目标框架 | .NET Framework 4.8（不使用 4.7.2 / 4.8.1 / .NET8 / .NET10） |
| 项目路径 | 全部英文路径，无中文字符 |
| VS 设置 | 关闭热重载、关闭自动升级 SDK 风格项目，避免串口资源残留 |
| 编译输出 | Release 编译后的 EXE 可直接拷贝至 Windows 工控机运行，无需额外安装运行库 |

---

## 【六、分步开发流程】

### 第 1 步：搭建 WinForms 项目框架

- 创建项目，设置 .NET Framework 4.8
- 搭建主窗体基本布局（`TableLayoutPanel` + `Panel` 分区）
- 放置串口配置栏控件（`ComboBox` ×2 + `Button` ×2 + `Label` 指示灯）
- 放置日志 `RichTextBox` + 手动指令输入区
- 放置状态 `DataGridView` + 底部 `StatusStrip`
- 右下区域放置运动控制 `Panel` + `TabControl`（3 个 `TabPage`：标定/参数/诊断）
- 所有控件命名规范：前缀+功能（如 `btnOpen`、`cmbPort`、`rtbLog`、`dgvJoints`）

### 第 2 步：实现串口收发模块

- 封装 `SerialPortHelper` 类：`Open()` / `Close()` / `Send(string)` / 事件通知
- 实现接收行缓冲（`StringBuilder` + `DataReceived` → 切行 → `BeginInvoke`）
- 实现日志写入方法（`AppendLog(string text, LogDirection dir)`：TX 用绿色、RX 用白色，带时间戳）
- 测试：手动发送 `"h\r\n"`，确认收到 help 文本并完整显示在日志区

### 第 3 步：实现状态轮询与解析

- 添加 200ms 定时器，Tick 内发送 `"s\r\n"`
- 实现 `ParseStatusResponse(string raw)` → 正则提取 State/Err/ESTOP/Motion + 6 关节数据 → 更新 `DataGridView`
- 实现状态枚举/错误码枚举 → UI 友好文本映射
- 从 "Zero Valid" 行解析零点状态 → 更新标定 Tab 和状态面板的零点指示灯
- 实现通信心跳检测（连续 5 次轮询无数据返回 → 报警）

### 第 4 步：实现运动控制按钮逻辑

- 单关节控制：组装 `"j{id} {angle} {speed}\r\n"` → 发送
- 多关节 PTP：组装 `"m {j1}...{j6} {speed}\r\n"` → 发送
- 一键指令按钮绑定对应的命令字符串（Zero/Ready/Home 回收/Home 部署）
- 急停按钮特殊处理：红色醒目样式（`BackColor = #E74C3C`, `ForeColor = White`, `Font.Bold`）+ 发送后锁定运动按钮
- 复位按钮：发送 `"clear"` 后恢复所有被锁定的按钮
- 夹爪控制：组装 `"g {angle} {servo_id}\r\n"` → 发送
- 角度输入框校验（限位 + J2/J3/J4 正值拦截）

### 第 5 步：实现 TabControl 三页签面板

**标定 Tab：**
- 标定流程按钮组：开始标定 → 确认设零 → 结束标定（三步顺序）
- 快捷设零按钮：独立关节选择 + 一键设零
- 零点状态指示灯面板：6 个彩色 Panel 或 Label
- 标定流程状态管理：未开始标定时"确认设零""结束标定"按钮置灰

**参数配置 Tab：**
- 关节选择下拉框切换时自动发送 `info {id}` 回读当前 PID/限位值并填入输入框
- PID 输入框 ×3 + 设置按钮
- 限速/限力矩输入框 + 设置按钮
- 模式切换单选按钮 + 切换按钮
- 保存/重启按钮（重启前弹确认框）

**诊断 Tab：**
- CAN/串口监控开关按钮组（每行：开启 + 关闭 + 状态指示）
- CAN ID 查询按钮
- CAN 硬件测试按钮组（回环测试 / 停止 / 寄存器诊断）
- 开启监控时提示 "将产生大量日志输出"

### 第 6 步：完善交互逻辑与防护

- 串口未连接 → 所有控制按钮 `Enabled = false`
- 急停锁定 → 复位解锁（监听日志 "ESTOP" / "cleared" 关键字切换锁定状态）
- J2/J3/J4 正值拦截弹窗（`MessageBox.Show("碰撞风险", "安全拦截", MessageBoxButtons.OK, MessageBoxIcon.Warning)`）
- 重启关节确认弹窗
- 窗口关闭 → 停止定时器 → 关闭串口 → 释放资源
- 所有 SerialPort 操作 `try-catch`，异常写入日志区

### 第 7 步：整体联调与优化

- 连接实际 MCU 测试全部 33 条命令
- UI 响应优化（`BeginInvoke` 批量更新、防抖）
- 日志清空/暂停/Hex 模式完善
- 通信断开自动检测 + 手动重连
- 诊断功能开启时自动暂停日志滚动（防止刷屏）
- 长时间运行稳定性测试（24h 日志累积 → 自动截断旧日志）

---

## 【七、AI 输出协作规则】

- 分步输出：按上述 7 步依次输出，每一步附带完整可编译代码、操作说明、功能说明
- 每步代码包含详尽的 XML 注释（`/// <summary>`）和行内注释
- 如果某步编译报错、串口通信异常、UI 卡顿等问题，主动提供完整排查流程与修复代码
- 所有新增功能严格匹配本文档需求，不擅自删减监测、控制、报警、串口交互相关功能
- 代码风格：工业规范，命名遵循 C# 惯例，不使用 `var`（显式类型），不引入不必要的依赖
- TabControl 的 3 个 TabPage 内容区在初始化时全部创建完毕，切换时不重建控件，仅刷新数据
