# 机械臂工控上位机 — 示教录制与曲线分析模块 AI 开发提示词 v1.0

（适配 VS2026 C# WinForms .NET Framework 4.8，作为主上位机的扩展 Tab 页签）

---

## 【AI 角色定位】

你是资深工控上位机开发工程师，在现有机械臂串口上位机工程基础上，新增一个"示教"TabPage，
实现关节运动数据的实时录制、曲线绘制、CSV 导出、回放复现与仿真对比功能。全程基于
Visual Studio 2026、C# WinForms .NET Framework 4.8，使用 WinForms 内置
`System.Windows.Forms.DataVisualization.Charting` 控件，不引入任何第三方图表库或 NuGet 包。

---

## 【前置依赖：已有工程架构】

本模块作为新增的 **第 4 个 TabPage** 集成到现有上位机的 TabControl 中：

```
现有 TabControl
  ├── Tab 1: 标定
  ├── Tab 2: 参数配置
  ├── Tab 3: 诊断
  └── Tab 4: 示教  ← 本次新增
```

### 已有数据源

上位机通过 200ms 周期定时器发送 `"s\r\n"` 轮询 MCU，解析返回文本后更新 UI。
`ParseStatusResponse(string raw)` 方法已解析出以下数据并存储在 UI 层变量中：

| 数据 | 类型 | 来源字段 | 示例 |
|------|------|----------|------|
| 系统状态 | `int` | State | 2 (RUNNING) |
| 错误码 | `int` | Err | 0 (NONE) |
| 急停标志 | `int` | ESTOP | 0 |
| 运动标志 | `int` | Motion | 1 |
| J1~J6 角度 | `float[6]` | Angle(°) | 0.00, -45.50, ... |
| J1~J6 转速 | `float[6]` | Speed(rpm) | 0.0, 30.0, ... |
| J1~J6 力矩 | `float[6]` | Torque(Nm) | 0.00, 1.20, ... |
| J1~J6 在线 | `bool[6]` | "ON"/"OFF" | true, true, ... |
| 零点有效 | `bool[6]` | J1=Y/J2=N | true, true, ... |
| 系统运行时间 | `uint32` | Uptime | 123456 (ms) |

### 已有串口通信

- `SerialPortHelper` 类：`IsOpen` / `Send(string)` / `DataReceived` 事件
- 发送示例：`serialPortHelper.Send("s\r\n")`
- 接收行缓冲已实现，通过 `ProcessLine(string line)` 分发解析

---

## 【一、新增模块功能需求】

### 功能总览

```
┌──────────────────────────────────────────────┐
│  【录制控制】                                  │
│  [▶ 开始录制] [⏹ 停止录制]                    │
│  状态: ●录制中  时长: 00:01:23  采样: 415 点  │
│                                              │
│  【实时曲线】  [角度 | 转速 | 力矩]             │
│  ┌──────────────────────────────────────┐    │
│  │  Chart 控件 — 多 Series 实时滚动曲线   │    │
│  │  J1=红 J2=蓝 J3=绿 J4=橙 J5=紫 J6=灰 │    │
│  │  X轴=时间(s)  Y轴=当前视图单位         │    │
│  └──────────────────────────────────────┘    │
│                                              │
│  【回放控制】                                  │
│  [▶ 回放] [⏸ 暂停] [⏹ 停止]                  │
│  速度: [0.5x] [1x] [2x] [5x]                 │
│  [▎▎▎▎▎░░░░░░░░░░]  进度: 45%  位置: 7.2s   │
│                                              │
│  【数据管理】                                  │
│  [📂 导出 CSV] [🖼 导出图表为图片]              │
│  [🗑 清空录制数据]                            │
│  CSV 文件大小: 12.4 KB   记录数: 415          │
└──────────────────────────────────────────────┘
```

---

### 功能一：数据录制

#### 触发方式

- 点击【▶ 开始录制】按钮 → 开始将每次轮询数据写入内存缓冲区
- 点击【⏹ 停止录制】按钮 → 停止写入，缓冲区数据保留

#### 录制数据结构

```csharp
/// <summary>
/// 单帧关节状态快照
/// </summary>
public struct JointSnapshot
{
    /// <summary>相对录制开始的毫秒偏移 (0 为录制起点)</summary>
    public double ElapsedMs;

    /// <summary>系统绝对运行时间 (来自 s 命令 Uptime 字段)</summary>
    public uint SysUptimeMs;

    /// <summary>J1~J6 角度 (°)</summary>
    public float[] Angles;

    /// <summary>J1~J6 转速 (r/min)</summary>
    public float[] Speeds;

    /// <summary>J1~J6 力矩 (Nm)</summary>
    public float[] Torques;

    /// <summary>J1~J6 在线标志</summary>
    public bool[] Online;

    /// <summary>系统状态</summary>
    public int State;

    /// <summary>错误码</summary>
    public int ErrorCode;
}
```

#### 录制规则

| 规则 | 说明 |
|------|------|
| 数据源 | 上位机 200ms 定时器 Tick 中，轮询 `s` 返回后已解析的当前值 |
| 触发条件 | 仅当【开始录制】按钮已被点击（`_isRecording == true`）时才追加 |
| 时间戳 | 录制开始时记录 `DateTime.Now` 基准，每帧 `ElapsedMs` = 当前时间 - 基准时间 |
| 内存管理 | 使用 `List<JointSnapshot>`，采样率 ~5Hz，每 100 秒约 500 帧，约 120KB 内存 |
| 上限保护 | 最大录制 3600 秒（1 小时，约 18000 帧，~4MB），超过提示"已达最大录制时长"并自动停止 |
| 录制指示灯 | 录制中在 Tab 页签标题旁显示红色圆点（如将 TabPage.Text 设为 `"示教 ●"`），停止后复原 |

#### 与现有轮询的集成点

在现有 200ms `Timer_Tick` 回调中，**状态解析完成后**增加：

```csharp
// === 现有代码 ===
ParseStatusResponse(rawResponse);
UpdateDataGridView();
UpdateStatusIndicators();

// === 新增：录制逻辑 ===
if (_isRecording)
{
    var snap = new JointSnapshot
    {
        ElapsedMs = (DateTime.Now - _recordStartTime).TotalMilliseconds,
        SysUptimeMs = _currentUptime,
        Angles = (float[])_currentAngles.Clone(),
        Speeds = (float[])_currentSpeeds.Clone(),
        Torques = (float[])_currentTorques.Clone(),
        Online = (bool[])_currentOnline.Clone(),
        State = _currentState,
        ErrorCode = _currentErrorCode
    };
    _recordingBuffer.Add(snap);

    // 实时更新 Chart
    AddSnapshotToChart(snap);

    // 检查录制时长上限
    if (snap.ElapsedMs > 3600_000)
    {
        StopRecording();
        AppendLog("[示教] 已达最大录制时长 (1小时)，自动停止录制");
    }
}
```

---

### 功能二：实时曲线显示

#### 控件选择

使用 WinForms 内置 `System.Windows.Forms.DataVisualization.Charting.Chart` 控件。

> **注意**：如果 VS2026 工具箱中没有 Chart 控件，需手动添加引用：
> 1. 解决方案资源管理器 → 引用 → 添加引用
> 2. 程序集 → 框架 → 勾选 `System.Windows.Forms.DataVisualization`
> 3. 工具箱 → 右键 → 选择项 → 浏览 → 选择 `System.Windows.Forms.DataVisualization.dll`

#### Chart 初始化配置

```csharp
/// <summary>
/// 初始化 Chart 控件：6 条 Series + X 轴时间 + Y 轴自适应 + 图例 + 网格
/// </summary>
private void InitChart(Chart chart)
{
    // 基础设置
    chart.ChartAreas.Clear();
    chart.Series.Clear();
    chart.Legends.Clear();

    var area = new ChartArea("AreaMain");
    area.AxisX.Title = "时间 (s)";
    area.AxisX.Minimum = 0;
    area.AxisX.MajorGrid.LineColor = Color.FromArgb(230, 230, 230);
    area.AxisY.Title = "角度 (°)";
    area.AxisY.MajorGrid.LineColor = Color.FromArgb(230, 230, 230);
    area.AxisY.IsStartedFromZero = false;  // Y 轴自适应范围
    chart.ChartAreas.Add(area);

    // 6 条关节曲线
    string[] names = { "J1", "J2", "J3", "J4", "J5", "J6" };
    Color[] colors = {
        Color.FromArgb(231, 76, 60),   // J1=红
        Color.FromArgb(52, 152, 219),  // J2=蓝
        Color.FromArgb(46, 204, 113),  // J3=绿
        Color.FromArgb(243, 156, 18),  // J4=橙
        Color.FromArgb(155, 89, 182),  // J5=紫
        Color.FromArgb(149, 165, 166)  // J6=灰
    };

    for (int i = 0; i < 6; i++)
    {
        var series = new Series(names[i]);
        series.ChartType = SeriesChartType.FastLine;  // 高性能折线图
        series.Color = colors[i];
        series.BorderWidth = 2;
        series.ChartArea = "AreaMain";
        chart.Series.Add(series);
    }

    // 图例
    var legend = new Legend("LegendMain");
    legend.Docking = Docking.Top;
    legend.Alignment = StringAlignment.Center;
    chart.Legends.Add(legend);

    // 交互
    chart.ChartAreas[0].CursorX.IsUserEnabled = true;       // 允许鼠标水平拖拽
    chart.ChartAreas[0].CursorX.IsUserSelectionEnabled = true;
    chart.ChartAreas[0].AxisX.ScaleView.Zoomable = true;    // 允许滚轮缩放
    chart.ChartAreas[0].AxisY.ScaleView.Zoomable = true;
}
```

#### 实时数据追加

```csharp
/// <summary>
/// 追加一帧数据到 Chart 的各 Series
/// </summary>
private void AddSnapshotToChart(JointSnapshot snap, bool isPlayback = false)
{
    double t = snap.ElapsedMs / 1000.0;  // 毫秒 → 秒

    // 根据当前视图模式选择数据源
    float[] data;
    switch (_chartViewMode)
    {
        case ChartViewMode.Speed:  data = snap.Speeds;  break;
        case ChartViewMode.Torque: data = snap.Torques; break;
        default:                   data = snap.Angles;  break;
    }

    for (int i = 0; i < 6; i++)
    {
        var series = chartMain.Series[i];
        series.Points.AddXY(t, data[i]);

        // 保持最近 150 个点显示 (≈30秒 @5Hz)，防止卡顿
        // 回放模式下不裁剪，保留全部
        if (!isPlayback && series.Points.Count > 150)
            series.Points.RemoveAt(0);
    }

    // 自动滚动 X 轴：X 轴最大值 = 最新时间戳 + 1 秒余量
    // 仅在非回放模式（实时录制 / 实时预览）下自动滚动
    if (!isPlayback && t > chartMain.ChartAreas[0].AxisX.Maximum - 1.0)
    {
        chartMain.ChartAreas[0].AxisX.Maximum = t + 1.0;
        chartMain.ChartAreas[0].AxisX.Minimum = Math.Max(0, t - 29.0);  // 显示最近 30 秒窗口
    }
}
```

#### 视图切换

三个单选按钮切换 Y 轴含义和标题：

| 按钮 | Y 轴标题 | 数据显示 |
|------|----------|----------|
| `rdoViewAngle` (默认) | 角度 (°) | `snap.Angles` |
| `rdoViewSpeed` | 转速 (r/min) | `snap.Speeds` |
| `rdoViewTorque` | 力矩 (Nm) | `snap.Torques` |

切换时清空 Chart 所有 Series 的 Points，然后用当前 `_recordingBuffer` 全量重绘。

#### 回放时的 Chart 行为

- 回放模式下不裁剪旧数据点（显示完整录制）
- 在 Chart 上叠加一条**垂直游标线**（`VerticalLineAnnotation`），标记当前回放位置
- 回放结束后游标线停留在末端

---

### 功能三：CSV 数据导出

#### 导出格式

```csv
# DaRan Robot Arm — Motion Recording
# Date: 2026-08-11 14:30:00
# Duration: 30.0s | Samples: 150 | Rate: 5Hz
# Command: z (HOME: all joints to 0°)
#
Time(s),SysUptime(ms),J1_Angle(deg),J1_Speed(rpm),J1_Torque(Nm),J1_Online,J2_Angle(deg),J2_Speed(rpm),J2_Torque(Nm),J2_Online,J3_Angle(deg),J3_Speed(rpm),J3_Torque(Nm),J3_Online,J4_Angle(deg),J4_Speed(rpm),J4_Torque(Nm),J4_Online,J5_Angle(deg),J5_Speed(rpm),J5_Torque(Nm),J5_Online,J6_Angle(deg),J6_Speed(rpm),J6_Torque(Nm),J6_Online,State,ErrorCode,Motion,ESTOP
0.000,123456,0.00,0.0,0.00,1,-45.50,0.0,1.20,1,-55.00,0.0,0.80,1,-70.00,0.0,0.50,1,110.00,0.0,0.30,1,0.00,0.0,0.10,1,2,0,0,0
0.200,123656,0.00,0.0,0.00,1,-44.80,5.0,1.25,1,-54.50,3.0,0.82,1,-69.80,2.0,0.52,1,110.00,0.0,0.30,1,0.00,0.0,0.10,1,2,0,1,0
0.400,123856,0.00,0.0,0.00,1,-43.20,12.0,1.30,1,-53.00,8.0,0.85,1,-69.00,5.0,0.55,1,110.00,0.0,0.30,1,0.00,0.0,0.10,1,2,0,1,0
...
```

#### 导出实现

```csharp
/// <summary>
/// 导出录制数据到 CSV 文件
/// </summary>
private void ExportCSV()
{
    if (_recordingBuffer.Count == 0)
    {
        MessageBox.Show("没有可导出的录制数据", "提示",
                        MessageBoxButtons.OK, MessageBoxIcon.Information);
        return;
    }

    using (SaveFileDialog sfd = new SaveFileDialog())
    {
        sfd.Filter = "CSV 文件 (*.csv)|*.csv|所有文件 (*.*)|*.*";
        sfd.FileName = $"DaRan_Motion_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
        sfd.DefaultExt = "csv";

        if (sfd.ShowDialog() != DialogResult.OK) return;

        using (StreamWriter sw = new StreamWriter(sfd.FileName, false, Encoding.UTF8))
        {
            // === 文件头注释 ===
            sw.WriteLine("# DaRan Robot Arm — Motion Recording");
            sw.WriteLine($"# Date: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            double duration = _recordingBuffer[_recordingBuffer.Count - 1].ElapsedMs / 1000.0;
            sw.WriteLine($"# Duration: {duration:F1}s | Samples: {_recordingBuffer.Count} | Rate: 5Hz");
            if (!string.IsNullOrEmpty(_lastCommandLabel))
                sw.WriteLine($"# Command: {_lastCommandLabel}");
            sw.WriteLine("#");

            // === 列标题 ===
            sw.WriteLine("Time(s),SysUptime(ms)," +
                         "J1_Angle(deg),J1_Speed(rpm),J1_Torque(Nm),J1_Online," +
                         "J2_Angle(deg),J2_Speed(rpm),J2_Torque(Nm),J2_Online," +
                         "J3_Angle(deg),J3_Speed(rpm),J3_Torque(Nm),J3_Online," +
                         "J4_Angle(deg),J4_Speed(rpm),J4_Torque(Nm),J4_Online," +
                         "J5_Angle(deg),J5_Speed(rpm),J5_Torque(Nm),J5_Online," +
                         "J6_Angle(deg),J6_Speed(rpm),J6_Torque(Nm),J6_Online," +
                         "State,ErrorCode,Motion,ESTOP");

            // === 数据行 ===
            foreach (var snap in _recordingBuffer)
            {
                sw.Write($"{snap.ElapsedMs / 1000.0:F3},{snap.SysUptimeMs},");
                for (int j = 0; j < 6; j++)
                {
                    sw.Write($"{snap.Angles[j]:F2},{snap.Speeds[j]:F1},{snap.Torques[j]:F2},{snap.Online[j] ? 1 : 0},");
                }
                sw.WriteLine($"{snap.State},{snap.ErrorCode},0,0");
            }
        }

        AppendLog($"[示教] CSV 已导出: {sfd.FileName} ({new FileInfo(sfd.FileName).Length / 1024} KB)");
    }
}
```

#### 图表导出为图片

```csharp
/// <summary>
/// 将当前 Chart 导出为 PNG 图片
/// </summary>
private void ExportChartImage()
{
    using (SaveFileDialog sfd = new SaveFileDialog())
    {
        sfd.Filter = "PNG 图片 (*.png)|*.png|JPEG 图片 (*.jpg)|*.jpg";
        sfd.FileName = $"DaRan_Chart_{DateTime.Now:yyyyMMdd_HHmmss}.png";
        if (sfd.ShowDialog() == DialogResult.OK)
        {
            chartMain.SaveImage(sfd.FileName, ChartImageFormat.Png);
            AppendLog($"[示教] 图表图片已导出: {sfd.FileName}");
        }
    }
}
```

---

### 功能四：回放复现

#### 回放机制

回放时不连接 MCU——从内存中的 `_recordingBuffer` 读取数据，使用独立的回放定时器按时间戳驱动。

```csharp
// 回放状态
private bool _isPlayback = false;
private bool _isPlaybackPaused = false;
private int _playbackIndex = 0;
private DateTime _playbackStartTime;
private double _playbackSpeed = 1.0;  // 1x, 2x, 5x 等
private System.Windows.Forms.Timer _playbackTimer;  // 50ms 周期回放定时器

/// <summary>
/// 回放定时器 Tick：取出下一帧数据，更新 Chart 游标 + 底部状态面板
/// </summary>
private void PlaybackTimer_Tick(object sender, EventArgs e)
{
    if (!_isPlayback || _isPlaybackPaused || _recordingBuffer.Count == 0)
        return;

    // 按回放速度计算应到达的虚拟位置 (ms)
    double elapsed = (DateTime.Now - _playbackStartTime).TotalMilliseconds * _playbackSpeed;

    // 找到第一个超过 elapsed 的帧
    while (_playbackIndex < _recordingBuffer.Count &&
           _recordingBuffer[_playbackIndex].ElapsedMs <= elapsed)
    {
        var snap = _recordingBuffer[_playbackIndex];

        // 更新 Chart 游标线位置
        UpdatePlaybackCursor(snap.ElapsedMs / 1000.0);

        // 更新回放进度条和标签
        double progress = (_playbackIndex + 1) / (double)_recordingBuffer.Count;
        progressBarPlayback.Value = (int)(progress * 100);
        lblPlaybackPosition.Text = $"{(snap.ElapsedMs / 1000.0):F1}s";
        lblPlaybackProgress.Text = $"{(progress * 100):F0}%";

        // 可选：将当前帧数据写入状态面板 (模拟实时数据显示)
        UpdateStatusPanelFromSnapshot(snap);

        _playbackIndex++;
    }

    // 回放结束
    if (_playbackIndex >= _recordingBuffer.Count)
    {
        StopPlayback();
        AppendLog("[示教] 回放完成");
    }
}
```

#### 回放控制按钮

| 按钮 | 行为 |
|------|------|
| 【▶ 回放】(`btnPlaybackStart`) | 开始回放：`_playbackIndex = 0` → 启动回放定时器 → Chart 全量显示完整录制数据 |
| 【⏸ 暂停】(`btnPlaybackPause`) | 暂停回放：`_isPlaybackPaused = true`，按钮文字变为 "▶ 继续" |
| 【⏹ 停止】(`btnPlaybackStop`) | 停止回放：重置 `_playbackIndex` → 移除游标线 → 进度条归零 |

#### 回放速度

| 速度 | 倍数 | 用途 |
|:---:|:---:|------|
| 0.5x | `_playbackSpeed = 0.5` | 慢动作分析，观察关节联动细节 |
| 1x | `_playbackSpeed = 1.0` | 真实速度回放 |
| 2x | `_playbackSpeed = 2.0` | 快速浏览 |
| 5x | `_playbackSpeed = 5.0` | 极速重放，快速定位 |

#### Chart 游标线

使用 `VerticalLineAnnotation` 在 Chart 上绘制一条竖直虚线标记当前回放位置：

```csharp
private VerticalLineAnnotation _playbackCursor;

private void InitPlaybackCursor()
{
    _playbackCursor = new VerticalLineAnnotation();
    _playbackCursor.AxisX = chartMain.ChartAreas[0].AxisX;
    _playbackCursor.ClipToChartArea = "AreaMain";
    _playbackCursor.IsInfinitive = false;
    _playbackCursor.LineColor = Color.Red;
    _playbackCursor.LineWidth = 2;
    _playbackCursor.LineDashStyle = ChartDashStyle.Dash;
    _playbackCursor.Visible = false;
    chartMain.Annotations.Add(_playbackCursor);
}

private void UpdatePlaybackCursor(double timeSeconds)
{
    _playbackCursor.X = timeSeconds;
    _playbackCursor.Visible = true;
}
```

#### 仿真对比（进阶功能）

在回放模式下，用户可加载一份参考 CSV（如 `traj_planner.py` 生成的理想轨迹），
与录制数据在同一 Chart 上叠加显示：

```
操作流程:
  1. 录制一段运动 → 导出 CSV (ground truth)
  2. 用 traj_planner.py 生成相同运动的理想轨迹 CSV
  3. 回放模式下 → [加载参考轨迹] → 选择理想轨迹 CSV
  4. Chart 上显示虚线 Series (理想) vs 实线 Series (实际)
  5. 可直观观察跟踪误差
```

实现方式：新增【加载参考轨迹】按钮 → 解析 CSV 文件 → 新增 6 条虚线 Series（命名如 "J1_ref"）→
使用不同线型（`BorderDashStyle = ChartDashStyle.Dash`）区分。

---

## 【二、UI 控件清单与布局】

### 新增控件列表

```
示教 TabPage (Text = "示教", Name = "tabTeach")
├── GroupBox "录制控制" (grpRecord)
│   ├── Button "▶ 开始录制" (btnRecordStart)   [BackColor=#27AE60, ForeColor=White]
│   ├── Button "⏹ 停止录制" (btnRecordStop)    [BackColor=#E74C3C, ForeColor=White]
│   ├── Label  状态文本 (lblRecordStatus)      [初始 "● 就绪", ForeColor=Gray]
│   ├── Label  时长 (lblRecordDuration)        [初始 "时长: --"]
│   └── Label  采样数 (lblRecordSamples)       [初始 "采样: 0 点"]
│
├── GroupBox "曲线视图" (grpChart)
│   ├── RadioButton "角度" (rdoViewAngle)      [Checked=true]
│   ├── RadioButton "转速" (rdoViewSpeed)
│   ├── RadioButton "力矩" (rdoViewTorque)
│   ├── Chart 控件 (chartMain)                 [Dock=Fill, 高度约 200px]
│   └── CheckBox "显示图例" (chkLegend)        [Checked=true]
│
├── GroupBox "回放控制" (grpPlayback)
│   ├── Button "▶ 回放" (btnPlaybackStart)
│   ├── Button "⏸ 暂停" (btnPlaybackPause)     [初始 Disabled]
│   ├── Button "⏹ 停止" (btnPlaybackStop)      [初始 Disabled]
│   ├── RadioButton "0.5x" (rdoSpeed05)
│   ├── RadioButton "1x"  (rdoSpeed1)          [Checked=true]
│   ├── RadioButton "2x"  (rdoSpeed2)
│   ├── RadioButton "5x"  (rdoSpeed5)
│   ├── ProgressBar (progressBarPlayback)
│   ├── Label 进度百分比 (lblPlaybackProgress)   [初始 "0%"]
│   └── Label 当前位置 (lblPlaybackPosition)     [初始 "0.0s"]
│
├── GroupBox "数据管理" (grpDataMgr)
│   ├── Button "📂 导出 CSV" (btnExportCSV)     [初始 Disabled]
│   ├── Button "🖼 导出图表" (btnExportChart)
│   ├── Button "🗑 清空数据" (btnClearData)     [初始 Disabled]
│   ├── Label 文件大小 (lblCSVSize)             [初始 "0 条记录"]
│   └── Button "加载参考轨迹" (btnLoadRef)      [回放时可用, 用于仿真对比]
│
└── GroupBox "命令标注" (grpCmdLabel)
    ├── TextBox (txtCmdLabel)                   [记录触发本次录制的命令, 如 "z" "ready"]
    └── Button "更新标注" (btnUpdateLabel)
```

### 按钮状态联动逻辑

| 状态 | btnRecordStart | btnRecordStop | btnExportCSV | btnClearData | btnPlayback* | rdoSpeed* |
|------|:---:|:---:|:---:|:---:|:---:|:---:|
| 初始（无数据） | ✅ | ❌ | ❌ | ❌ | ❌ | ✅ |
| 录制中 | ❌ | ✅ | ❌ | ❌ | ❌ | ✅ |
| 录制完成（有数据） | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ |
| 回放中 | ❌ | ❌ | ✅ | ❌ | ✅ | ❌ |
| 回放暂停 | ❌ | ❌ | ✅ | ❌ | ✅ | ❌ |
| 串口未连接 | ❌ | ❌ | ✅* | ✅* | ✅* | ✅* |

> `*` = 有历史数据则可用

---

## 【三、交互规则与边界条件】

| 场景 | 行为 |
|------|------|
| 开始录制时已有旧数据 | 弹出确认框："当前有未导出的录制数据，是否覆盖？" |
| 录制中切换视图 | 清空 Chart 重新用缓冲区全量绘制（不影响录制缓冲区） |
| 录制中离开示教 Tab | 录制继续在后台进行，切换回来时 Chart 全量刷新 |
| 回放中录制 | 不允许：回放中【开始录制】按钮置灰 |
| 录制中回放 | 不允许：录制中【回放】按钮置灰 |
| 清空数据 | 弹出确认框 → 清空 `_recordingBuffer` → Chart.Series.Clear() → 重置状态 |
| 窗口关闭时有未保存数据 | `FormClosing` 中检查 `_recordingBuffer.Count > 0` → 提示是否导出后再关闭 |
| CSV 导出文件覆盖 | 标准 `SaveFileDialog` 自动处理覆盖确认 |

---

## 【四、命令标注功能】

在教学场景中，每段录制通常对应一次特定的运动命令。`txtCmdLabel` 允许用户标注：

```
录制时：【▶ 开始录制】→ 自动填入最近一次从运动面板发送的命令字符串
         例: 用户点了 [Zero] 按钮 → 标注自动填入 "z (HOME: all joints to 0°)"

手动修改：用户可在输入框中自由编辑标注内容
CSV 导出：标注内容写入 CSV 文件头注释的 # Command: ... 行
```

实现：在 `SerialPortHelper.Send()` 方法中记录最后一次发送的命令字符串到公共变量 `LastSentCommand`，
录制开始时自动读取该变量填入标注框。

---

## 【五、开发步骤】

### 第 1 步：创建示教 TabPage + 基础数据结构

- 在现有 `TabControl` 中新增 `TabPage`：`Name = "tabTeach"`, `Text = "示教"`
- 定义 `JointSnapshot` 结构体
- 添加成员变量：`List<JointSnapshot> _recordingBuffer`、`bool _isRecording`、`DateTime _recordStartTime`
- 放置 GroupBox + Button 控件骨架

### 第 2 步：实现录制功能

- 实现 `btnRecordStart_Click` / `btnRecordStop_Click`
- 在现有 200ms 定时器 `Timer_Tick` 中集成录制追加逻辑
- 实现录制时长/采样数标签更新
- 实现录制上限自动停止（3600 秒）
- 测试：连接 MCU，点开始录制 → 发送几条运动命令 → 停止录制 → 检查 `_recordingBuffer.Count`

### 第 3 步：实现 Chart 实时曲线

- 添加 `System.Windows.Forms.DataVisualization` 引用
- 放置 Chart 控件，实现 `InitChart()`
- 实现 `AddSnapshotToChart()` 实时追加方法
- 实现视图切换（角度/转速/力矩）三个 RadioButton
- 实现最近 30 秒滑动窗口 + 自动 X 轴滚动
- 测试：录制一段运动，观察 6 条曲线实时滚动

### 第 4 步：实现 CSV + 图片导出

- 实现 `ExportCSV()`：文件头注释 + 列标题 + 数据行
- 实现 `ExportChartImage()`：`chartMain.SaveImage()`
- 实现 `btnClearData_Click`：确认框 → 清空 Chart + 缓冲区
- 测试：录制 → 导出 CSV → 用 Excel 打开验证格式

### 第 5 步：实现回放功能

- 添加回放控制按钮组 + 进度条
- 实现回放定时器（50ms 周期）+ `PlaybackTimer_Tick`
- 实现 `VerticalLineAnnotation` 游标线
- 实现回放速度切换
- 实现回放时的状态面板同步更新
- 测试：录制一段运动 → 停止 → 回放 1x → 观察游标线移动

### 第 6 步：实现仿真对比

- 实现【加载参考轨迹】按钮 + `OpenFileDialog` 解析 CSV
- 解析参考 CSV 到独立 `List<JointSnapshot>`
- 新增 6 条虚线 Series（J1_ref ~ J6_ref）叠加显示
- 测试：录制 → 导出 CSV → 修改几行模拟误差 → 重新加载作为参考 → 观察实线 vs 虚线差异

### 第 7 步：完善交互与体验

- 按钮状态联动逻辑（录制/回放互斥等）
- 命令标注自动填入
- 窗口关闭未保存提示
- 长时间录制内存/性能测试
- 录制中 Tab 页签标题红点指示

---

## 【六、Chart 高级配置速查】

### 外观优化

```csharp
// 抗锯齿
chart.AntiAliasing = AntiAliasingStyles.All;

// 背景色
chart.BackColor = Color.FromArgb(250, 250, 250);
area.BackColor = Color.White;

// 网格线
area.AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dot;
area.AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dot;

// 图例样式
legend.Font = new Font("Segoe UI", 9f);
legend.BackColor = Color.Transparent;

// Y 轴自动范围时保留 10% 边距
area.AxisY.Margin = 10;  // 10% padding

// X 轴格式化为秒
area.AxisX.LabelStyle.Format = "{0:F0}s";
```

### 鼠标悬停显示数值

```csharp
// ToolTip 显示 Series 名称和数值
chart.GetToolTipText += (sender, e) =>
{
    if (e.HitTestResult.ChartElementType == ChartElementType.DataPoint)
    {
        var point = e.HitTestResult.Series.Points[e.HitTestResult.PointIndex];
        e.Text = $"{e.HitTestResult.Series.Name}\n时间: {point.XValue:F1}s\n值: {point.YValues[0]:F2}";
    }
};
```

---

## 【七、CSV 文件格式完整规范】

### 文件头（以 `#` 开头的注释行，可被 Python/MATLAB 忽略）

```
# DaRan Robot Arm — Motion Recording
# Date: 2026-08-11 14:30:00
# Duration: 30.0s | Samples: 150 | Rate: 5Hz | MaxJoints: 6
# Command: z (HOME: all joints to 0°)
#
```

### 列定义（按序共 29 列）

```
列序  列名              类型    单位    说明
 0    Time              float   s       相对录制起点的秒数
 1    SysUptime         uint    ms      MCU 系统运行时间
 2    J1_Angle          float   deg     J1 关节角度
 3    J1_Speed          float   rpm     J1 转速
 4    J1_Torque         float   Nm      J1 力矩
 5    J1_Online         int     0/1     J1 在线标志
 6    J2_Angle          float   deg
 7    J2_Speed          float   rpm
 8    J2_Torque         float   Nm
 9    J2_Online         int     0/1
10    J3_Angle          float   deg
11    J3_Speed          float   rpm
12    J3_Torque         float   Nm
13    J3_Online         int     0/1
14    J4_Angle          float   deg
15    J4_Speed          float   rpm
16    J4_Torque         float   Nm
17    J4_Online         int     0/1
18    J5_Angle          float   deg
19    J5_Speed          float   rpm
20    J5_Torque         float   Nm
21    J5_Online         int     0/1
22    J6_Angle          float   deg
23    J6_Speed          float   rpm
24    J6_Torque         float   Nm
25    J6_Online         int     0/1
26    State             int     enum    0=INIT,1=READY,2=RUNNING,3=ESTOP,4=ERROR,6=ZERO_CHECK,7=CALIB
27    ErrorCode         int     enum    见 app_threads.h 错误码枚举
28    Motion            int     0/1     运动中标志
29    ESTOP             int     0/1     急停标志
```

### Python 读取示例（供教学参考）

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV（跳过 # 注释行）
df = pd.read_csv("DaRan_Motion_20260811_143000.csv", comment='#')

# 绘制 6 关节角度曲线
fig, axes = plt.subplots(2, 3, figsize=(14, 8))
for i, ax in enumerate(axes.flat):
    ax.plot(df['Time(s)'], df[f'J{i+1}_Angle(deg)'], linewidth=1.5)
    ax.set_title(f'Joint {i+1}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (°)')
    ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# 与理想轨迹对比（如有时）
df_ref = pd.read_csv("ideal_trajectory.csv")
error = df['J1_Angle(deg)'] - df_ref['J1_Angle(deg)']
print(f"J1 RMS error: {((error ** 2).mean()) ** 0.5:.4f}°")
```

---

## 【八、安全与稳定性注意事项】

| 注意点 | 说明 |
|------|------|
| 录制不影响控制 | 录制逻辑在定时器回调末尾执行，不阻塞串口收发，不影响 MCU 通信 |
| 回放不发送指令 | 回放模式只读取内存数据更新 UI，**绝不向串口发送任何命令** |
| Chart 线程安全 | Chart 数据追加通过 `BeginInvoke` 切换到 UI 线程 |
| 内存上限 | 最大录制 1 小时硬限制，防止内存溢出 |
| 窗口关闭保护 | `FormClosing` 中检查未保存数据，防止误关丢失 |
| CSV 编码 | 统一 UTF-8 with BOM，确保 Excel 直接打开中文不乱码 |

---

## 【九、AI 输出协作规则】

- 分步输出：按上述 7 步依次输出，每一步附带完整可编译代码
- 每步代码包含 XML 注释（`/// <summary>`）和行内注释
- 明确标注新增代码 vs 修改现有代码的位置（如"在 `Timer_Tick` 末尾增加以下代码块"）
- 如果 Chart 控件不可用、编译报错等问题，主动提供排查步骤
- 代码风格与主上位机项目保持一致：显式类型、C# 命名惯例、不引入第三方依赖
