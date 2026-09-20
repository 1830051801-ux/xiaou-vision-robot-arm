# STM32 and ROS 2 Joint Bring-up Plan

This plan is for the first joint-controller integration session. It is
deliberately staged so a protocol or sign mistake cannot move the arm
unexpectedly. The Pi must be running the verified Python 3.12 ROS 2
environment, and the physical emergency stop must be reachable before any
actuator is powered.

## Known facts and unresolved values

- CAN application bus: Classic CAN, 11-bit identifiers, 500 kbit/s, DLC 8.
- Provisional node IDs: J1..J6 = 1..6. Confirm passively; do not assume them.
- From each motor gear side, a positive motor position command rotates the gear
  counter-clockwise. This is stored as `motor_positive_rotation=gear_ccw`.
- ROS encoder sign, encoder zero, limits, feedback timing, and STM32 command
  semantics remain unmeasured. They must stay locked in the calibration JSON.
- The six-axis CAD model is used for offline FK/IK and collision review only;
  it is not evidence of physical zero or limits.

## Stage 0: software-only checks

1. Run `scripts/verify_pi_environment.sh` with CAN disconnected.
2. Run the Python CAN protocol tests and loopback tests.
3. Confirm `motion_enabled=false`, all verification gates false, and null
   encoder/limit fields. Do not run a launch file that loads the CAN hardware.

## Stage 1: STM32 loopback, no motor power

1. STM32 accepts only standard 11-bit frames with DLC 8.
2. Command frame: `0x100 + node_id`, byte 0 `0x01`, byte 1 flags/sequence,
   bytes 2..5 signed little-endian position in micro-radians, bytes 6..7
   signed little-endian velocity in milli-radians/s.
3. Feedback frame: `0x180 + node_id`, byte 0 `0x81`, status byte, position and
   velocity with the same scaling.
4. Verify byte order, sign extension, sequence handling, quick-stop behavior,
   unknown-opcode rejection, and the 200 ms command/feedback watchdog.
5. Use a virtual CAN or a disconnected STM32 test fixture. No actuator output.

## Stage 2: passive bus identification

1. Connect the CAN transceiver but leave motor power disabled.
2. Pi listens only; it must not transmit position, enable, clear-fault, or
   quick-stop frames during identification.
3. Record each real feedback ID, status, firmware version/diagnostic data, and
   map it to J1..J6 only after the mechanical harness is confirmed.

## Stage 3: one-axis sign and zero calibration

For one unloaded joint at a time:

1. Fit a physical reference mark and record the encoder reading at that pose.
2. With a reachable E-stop and no payload, command at most 1--2 degrees and
   no more than 5% of the final speed.
3. Confirm the gear-side observation: positive motor command is counter-clockwise.
4. Compare the encoder feedback delta with the ROS positive-axis convention;
   write `encoder_direction` only after three repeatable observations.
5. Return to the reference mark three times and average the measured encoder
   value for `encoder_zero_offset_rad`.
6. Determine soft limits well inside the mechanical hard stops, then record
   velocity and acceleration limits.
7. Repeat for J1 through J6. Any disagreement stops the session.

## Stage 4: guarded ROS 2 integration

1. Populate the calibration JSON only with measured values.
2. Update the matching URDF/ros2_control joint parameters and rebuild the
   workspace.
3. Enable protocol, feedback, E-stop, and motion gates one at a time; verify
   that an absent feedback frame disables the hardware within 200 ms.
4. Start with a single joint, a zero-velocity hold, and a small position step.
   Then test a slow two-point trajectory. Only after all six axes pass may the
   planner send a trajectory.

## UART/debug channel

Use STM32 UART only for firmware logs, boot state, node-ID reporting, and error
codes unless the firmware specification explicitly defines UART as the actuator
command transport. The ROS 2 hardware plugin and the documented six-axis
position protocol use CAN; do not silently substitute UART framing for CAN.

## Evidence to save

Save the exact STM32 firmware commit, CAN bitrate configuration, passive capture,
each joint's ID/sign/zero/limits table, E-stop test result, and the output of
`verify_pi_environment.sh`. No motion gate is considered complete without these
artifacts.
