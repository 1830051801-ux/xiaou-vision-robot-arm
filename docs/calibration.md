# 标定

小U的视觉定位使用固定相机下的九点平面单应性。标定结果必须声明图像分辨率、输出坐标系和长度单位，不能把旧分辨率或旧桌面高度直接复用到新设备。

## 数据合同

- 输入：九个不共线桌面点的像素坐标与 `robot_base_table` 平面坐标。
- 当前规划器相机合同：`1920 x 1080`，输出单位为米。
- 输出：3×3 homography、采集时间、点集误差、相机和桌面标识。
- 运行时只读取已保存的 homography，不读取仿真场景坐标来代替相机结果。

## 离线检查

```powershell
Push-Location raspberry_pi
python tools/audit_homography_workspace.py
python -m unittest discover -s tests -p 'test_*homography*.py' -q
Pop-Location
```

最新工作簿中的 27 个点、均值重投影误差和最大误差见
[`evidence/field_validation_20260923/README.md`](evidence/field_validation_20260923/README.md)。这些数值是现场工作簿记录，不是本命令重新采集的结果。

## 上机前复核

1. 固定相机支架和桌面，确认分辨率与曝光设置。
2. 重新采集九点并记录点序、平面高度和坐标方向。
3. 检查均值/最大误差、工作空间边界和异常点，再替换站点配置。
4. 先执行只读视觉和坐标预览，最后由现场流程决定是否开放运动。
