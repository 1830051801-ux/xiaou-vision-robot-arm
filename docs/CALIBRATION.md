# Camera Calibration

The Raspberry Pi converts image detections to robot base coordinates through the workspace homography file:

```text
raspberry_pi/runtime/calibration/workspace_homography.yaml
```

Calibration should be repeated when the camera, camera mount, work table or arm base changes.

Recommended check sequence:

```bash
bash scripts/run_camera_test.sh
bash scripts/run_yolo_test.sh
bash scripts/run_demo_all.sh --object pen
```

Review the dry-run output before using `--send-serial`:

- detected object name
- pixel center
- base-frame `x_base_mm`, `y_base_mm`
- generated UART frame

The STM32 firmware also checks radial distance and J1 angle before moving.

