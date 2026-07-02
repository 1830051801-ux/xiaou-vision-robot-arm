# 小U树莓派使用与联调说明

## 1. 工程定位

树莓派端承担小U的多模态感知与交互层：

- 语音输入：接收用户自然语言指令。
- 云端对话：生成小U回应，并根据语义触发视觉抓取流程。
- 表情显示：高清屏上显示小U状态与情绪。
- 视觉识别：调用自训练YOLO模型识别笔、杯子、可乐、水瓶、耳机。
- 坐标解算：结合九点标定，把画面中的物体位置换算为机械臂基坐标。
- 坐标下发：通过串口向STM32发送抓取目标坐标。

对外展示时可以这样解释：

> 小U通过摄像头看到桌面物体，经过自训练AI模型识别和标定坐标换算，把目标位置发送给底层控制器，由机械臂完成抓取。

## 2. 树莓派端启动

进入工程并激活环境：

```bash
cd ~/raspi_robot_ai
source .venv/bin/activate
```

启动小U演示：

```bash
DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 bash scripts/run_demo_all.sh
```

启动小U并打开YOLO预览框：

```bash
XIAOU_OPEN_YOLO=1 DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 bash scripts/run_demo_all.sh
```

关闭旧进程：

```bash
pkill -f "robot_ai/chat_window.py" 2>/dev/null || true
pkill -f "robot_ai/02_yolo_detect.py" 2>/dev/null || true
```

## 3. 常用演示说法

### 让小U观察桌面

用户可以说：

- “小U，你看到什么东西了？”
- “帮我看一下桌面上有什么。”

期望行为：

- 小U打开视觉识别。
- 识别桌面物体。
- 回答物体类别、位置和大概坐标。

### 让小U拿物体

用户可以说：

- “小U，帮我拿笔。”
- “帮我拿饮料。”
- “帮我把水瓶拿过来。”
- “帮我整理一下桌面。”

期望行为：

- 小U先识别目标物。
- 视觉端输出物体中心点、角度、尺寸。
- 标定模块换算为机械臂基坐标。
- 树莓派通过串口发送坐标帧。
- STM32控制机械臂执行抓取。

## 4. 关键文件说明

### `robot_ai/chat_window.py`

小U主程序入口，负责：

- 启动语音交互。
- 控制小U显示窗口。
- 接入视觉识别结果。
- 根据用户指令触发抓取流程。

### `robot_ai/emotion_state.py`

小U人设和情绪逻辑，负责：

- 自我介绍只说一次。
- 正常对话后回到待机状态。
- 根据用户语义切换开心、思考、疑问、委屈等表情。
- 让回答更短、更像桌面机器人助手。

### `robot_ai/07_cloud_chat.py`

云端对话和TTS语音输出，负责：

- 调用对话模型。
- 生成自然中文回答。
- 使用Edge-TTS缓存常用语音，减少等待时间。

### `robot_ai/robot_protocol.py`

树莓派到STM32的坐标帧打包，负责：

- 把 `x_base_mm`、`y_base_mm`、`z_mm`、`theta_deg` 等结果打包成二进制帧。
- 保证帧头为 `0xAA`，帧尾为 `0x55`。
- 坐标采用 `int16 x 10` 的小端格式。
- 调试阶段CRC可以使用 `FF FF`。

### `robot_ai/vision_targeting.py`

视觉目标处理，负责：

- 筛选YOLO检测框。
- 判断目标中心点、宽度和方向。
- 限制抓取工作区，避免明显不合理坐标进入机械臂。
- 输出给协议层使用的基坐标。

### `codex_pickup_package/workspace_homography.yaml`

九点标定结果，负责：

- 保存像素坐标到机械臂基坐标的单应矩阵。
- 正式演示前不要随便覆盖。
- 如果机械臂、相机、桌面位置发生变化，需要重新标定。

## 5. 关键配置

配置文件：

`config.demo.env`

建议保留的关键项：

```bash
XIAOU_PROACTIVE_VISION=1
XIAOU_PROACTIVE_INTERVAL_S=35
XIAOU_PROACTIVE_COOLDOWN_S=75
```

含义：

- `XIAOU_PROACTIVE_VISION`：允许小U定时扫一眼桌面。
- `XIAOU_PROACTIVE_INTERVAL_S`：主动感知间隔。
- `XIAOU_PROACTIVE_COOLDOWN_S`：避免小U短时间重复说话。

## 6. 串口联调检查

树莓派侧常用检查：

```bash
ls -l /dev/serial0
python robot_ai/pick_pen_demo.py --dry-run --timeout 10
```

看到类似信息说明视觉和打包链路已经工作：

```text
PEN FOUND
Position: X=..., Y=...
MCU frame: AA 14 14 ...
```

重点看：

- `MCU frame` 第一个字节必须是 `AA`。
- 最后一个字节必须是 `55`。
- X/Y/Z不能异常为0，尤其Y坐标不能丢失。
- 坐标方向应符合当前机械臂基坐标定义。

## 7. 显示屏与桌面问题

查看HDMI连接：

```bash
cat /sys/class/drm/card1-HDMI-A-1/status 2>/dev/null
cat /sys/class/drm/card1-HDMI-A-2/status 2>/dev/null
cat /sys/class/drm/card1-HDMI-A-2/modes 2>/dev/null | head
```

查看桌面进程：

```bash
ps -ef | grep -E "labwc|wayfire|Xwayland|chat_window" | grep -v grep
ls -l /run/user/1000/wayland-*
```

如果屏幕黑但HDMI connected：

- 先确认显示屏供电稳定。
- 确认当前桌面会话是 `wayland-0`。
- 重新启动小U前先关闭旧进程。
- 避免同时开多个VNC或远程桌面占用显示。

## 8. 安全清理范围

可以清理：

```bash
find . -type d -name "__pycache__" -prune -exec rm -rf {} +
find . -name "*.pyc" -delete
```

不要随便删除：

- `models/`
- `config.demo.env`
- `codex_pickup_package/workspace_homography.yaml`
- `my_all_data/`
- `my_data/`
- `my_objects_data/`
- `runtime/tts_cache/`，除非需要重新生成语音缓存。

## 9. Windows上传核心文件

只上传小U核心文件，不覆盖整个工程：

```powershell
cd "C:\Users\ZhuanZ（无密码）\Desktop\桌面整理_2026-07-01_033319\raspi_robot_ai"
powershell -ExecutionPolicy Bypass -File ".\WINDOWS_上传小U核心文件.ps1" -PiIp 100.127.160.77 -User pi
```

如果Tailscale不可用，可以把 `-PiIp` 换成同一WiFi下的树莓派IP，例如 `172.20.10.3`。
