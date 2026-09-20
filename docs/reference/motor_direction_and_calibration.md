# Motor Direction and Calibration Record

## Confirmed motor-side convention

For J1 through J6, when looking at the corresponding motor gear from its gear
side, a positive motor position command produces counter-clockwise rotation.
This is recorded as `motor_positive_rotation: gear_ccw` in
`robot_ai/arm_control/config/hardware_calibration.json`.

This fact is a motor/command observation. It does **not** by itself determine
the ROS joint `direction` or the encoder feedback sign. Those require a
feedback observation on the real STM32 protocol: command a small positive
position change, read the encoder change, and record the sign while the axis is
unloaded and an emergency stop is reachable.

## Current software state

- Node IDs `1..6` are user-confirmed for this wiring: J1=1 through J6=6. The
  STM32 hardware-enabled target must still verify that each physical node
  answers on the expected ID before enabling motion.
- Encoder zero offsets remain `null` until captured from stationary telemetry.
- Encoder directions remain `null`.
- Position, velocity, and acceleration limits remain `null`.
- `motion_enabled`, protocol confirmation, E-stop verification, and feedback
  verification remain false.
- Therefore the real-motion gate remains closed.

## One-click pose capture

After flashing a hardware-enabled F407 image that reports real six-axis
feedback, physically hold the arm still in the desired reference pose and run:

```bash
cd /home/pi/raspi_robot_ai
python3 robot_ai/arm_control/capture_pose.py zero --write --confirm CAPTURE-ZERO
```

Then place the arm in the desired non-motion ready pose and run:

```bash
python3 robot_ai/arm_control/capture_pose.py ready --write --confirm CAPTURE-READY
```

The script only reads `GET_STATE`; it never sends a trajectory, CAN command, or
PWM command. The zero capture is a Pi-side model-feedback snapshot, not the
authoritative raw encoder offset: F407 must persist raw offsets and directions
with a version/CRC record. It refuses to write unless all six joints report
online, the arm is stationary, and the exact confirmation string is supplied.
The current transport-only F407 image intentionally cannot provide the required
real feedback, so a failed capture at this stage is expected.

## STM32 bring-up order

1. Implement and test the documented CAN frame format in a virtual CAN or
   disconnected loopback setup.
2. Passively verify the user-confirmed node IDs 1..6; do not transmit motion.
3. With one unloaded joint selected, use a positive command of at most 1--2
   degrees at no more than 5% speed. Confirm the gear-side counter-clockwise
   observation and the encoder feedback sign.
4. Repeat three times per joint, then write the measured encoder direction and
   zero offset. Measure soft limits before enabling trajectory planning.
5. Only after protocol, feedback, E-stop, limits, and IDs are independently
   recorded may the motion gates be changed.
