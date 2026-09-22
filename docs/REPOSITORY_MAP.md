# Repository map

小U仓库按运行边界组织。硬件入口、训练入口和现场证据互相独立，便于在没有设备时完成源码检查和回放。

```text
configs/default.yaml                         declared Pi/F407 profile
raspberry_pi/robot_ai/vision/                YOLO/ONNX adapters and registry
raspberry_pi/robot_ai/decision/              Temporal policy, replay and quantization
raspberry_pi/robot_ai/arm_control/           UART CRC16, calibration, kinematics and safety
raspberry_pi/simulation/                     desktop scene and numerical rollouts
raspberry_pi/ros2_ws/src/                    ROS 2 descriptions, MoveIt and typed previews
stm32_keil/BasicSetting_DaRanRobot/          F407 Keil project, UART parser and CAN threads
docs/evidence/field_validation_20260923/     22-sheet field workbook archive
scripts/                                     release checks and evidence import tools
raspberry_pi/tests/                          offline protocol/model/unit tests
```

## Requested component mapping

| Component | Source entry | Contract |
| --- | --- | --- |
| VLA inference | `raspberry_pi/robot_ai/decision/transformer_policy.py` | RGB/state/instruction observation, bounded action chunk, safety gate |
| YOLO wrapper | `raspberry_pi/robot_ai/vision/unified_yolo.py` | one detection schema for ONNX/OpenCV backends |
| UART + CRC16 | `raspberry_pi/robot_ai/arm_control/uart_protocol.py` | `0xAA ... CRC16/MODBUS ... 0x55`, escaping and sequence matching |
| Nine-point calibration | `raspberry_pi/robot_ai/arm_control/task_planner.py` and `raspberry_pi/codex_pickup_package/` | homography projection with frame/unit checks |
| Motion planning | `raspberry_pi/robot_ai/arm_control/trajectory.py`, `kinematics.py` | bounded joint/cartesian planning and replay |
| Safety monitor | `raspberry_pi/robot_ai/arm_control/safety.py` | motion lock, freshness, limits and fail-closed admission |
| MuJoCo | `raspberry_pi/simulation/` and `research/embodied-arm-learning/` | deterministic offline scene/replay paths |
| STM32 protocol | `stm32_keil/BasicSetting_DaRanRobot/Inc/comm_protocol.h` and `Src/comm_protocol.c` | frame parser, ACK/NACK and command dispatch |
| ROS 2 / MoveIt | `raspberry_pi/ros2_ws/src/` | descriptions, planning preview and dry-run status topics |

## Evidence flow

The latest workbook is copied byte-for-byte under `docs/evidence/field_validation_20260923/source/`. The importer also writes a complete sheet export and a hash manifest. Field records, software replay, and simulation results remain separate evidence classes.

## Maintainer checks

```powershell
py -3.13 -B scripts\verify_release.py
Push-Location raspberry_pi
python -m compileall -q robot_ai tools simulation tests
python -m unittest discover -s tests -q
Pop-Location
```

`verify_release.py` also checks the current field-workbook manifest and every
derived-file hash, so a partial or modified evidence bundle fails the release
check before it is published.

The public configuration keeps `motion_enabled: false`. Any site-specific
enablement must live outside the public repository and be accompanied by its
own firmware hash, calibration record, and operator sign-off.
