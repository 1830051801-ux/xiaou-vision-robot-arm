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

- Node IDs `1..6` remain provisional and must be passively confirmed.
- Encoder zero offsets remain `null`.
- Encoder directions remain `null`.
- Position, velocity, and acceleration limits remain `null`.
- `motion_enabled`, protocol confirmation, E-stop verification, and feedback
  verification remain false.
- Therefore the real-motion gate remains closed.

## STM32 bring-up order

1. Implement and test the documented CAN frame format in a virtual CAN or
   disconnected loopback setup.
2. Listen only and identify the six real node IDs; do not transmit motion.
3. With one unloaded joint selected, use a positive command of at most 1--2
   degrees at no more than 5% speed. Confirm the gear-side counter-clockwise
   observation and the encoder feedback sign.
4. Repeat three times per joint, then write the measured encoder direction and
   zero offset. Measure soft limits before enabling trajectory planning.
5. Only after protocol, feedback, E-stop, limits, and IDs are independently
   recorded may the motion gates be changed.
