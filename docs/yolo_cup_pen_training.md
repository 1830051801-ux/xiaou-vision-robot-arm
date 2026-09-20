# cup / bottle / pen YOLO 训练说明

这份配置是给小 U 的视觉识别用的，目标是把 `cup`、`bottle`、`pen` 三类训稳一点。

## 目标

- `cup`
- `bottle`
- `pen`

先把这三类做扎实，再考虑继续扩类。

## 数据建议

每一类至少准备 200 张起步，理想是 500 张以上。

采样时注意：

- 俯视、斜视、侧视都要有
- 亮桌面、暗桌面都要有
- 近距离大目标、远距离小目标都要有
- 遮挡、反光、手拿、背景杂乱都要有
- 空瓶、透明杯、细笔、黑笔、白笔都尽量覆盖

标注时注意：

- `cup` 不要把整张桌子框进去
- `bottle` 要把瓶身完整框住
- `pen` 尽量沿笔的真实长度框住，不要框成很大的正方形
- 同一张图里多个实例都标出来

## 推荐目录

```text
dataset/
  images/
    train/
    val/
    test/
  labels/
    train/
    val/
    test/
```

数据集 YAML 示例：

```yaml
path: D:/datasets/cup_pen
train: images/train
val: images/val
test: images/test
names:
  0: cup
  1: bottle
  2: pen
```

## 训练脚本

先生成模板：

```bash
cd C:\Users\ZhuanZ（无密码）\Desktop\raspi_robot_ai
python codex_pickup_package/train_yolo_cup_pen.py --write-template --data codex_pickup_package/cup_pen_dataset.yaml
```

然后修改 `path` 为你的真实数据集目录，再开始训练：

```bash
python codex_pickup_package/train_yolo_cup_pen.py ^
  --data codex_pickup_package/cup_pen_dataset.yaml ^
  --model yolov8n.pt ^
  --epochs 120 ^
  --imgsz 960 ^
  --batch 16
```

## 为什么这样配

- `imgsz=960`：细长的 `pen` 更吃分辨率
- `mosaic/mixup/copy_paste`：增强小样本泛化
- `degrees/translate/scale/shear`：让杯子和笔在桌面姿态变化下更稳
- `close_mosaic`：训练后期收敛更平稳

## 训练完成后

把最佳权重导出成 ONNX，再替换到 `models/` 目录里，推理侧默认会继续走 OpenCV DNN。

建议优先替换：

- `models/yolov8n.onnx`，如果你后面打算切到 YOLOv8
- 或保留现有 `yolov5n.onnx` 做兼容测试

## 推理侧已配合的改动

- 默认目标里已经补了 `pen`
- `cup` 和 `bottle` 仍然走更宽松的抓取姿态
- `pen` 默认走顶抓，不会把它当成杯子那种侧抓逻辑

