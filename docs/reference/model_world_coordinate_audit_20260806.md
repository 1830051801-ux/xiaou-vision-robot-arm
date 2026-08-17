# Six-axis model and world-coordinate audit

Date: 2026-08-06
Scope: offline only. No SocketCAN interface was opened and no actuator command was sent.

## Verified internal model

The STEP-derived POE model and the URDF reproduce each other within:

- maximum space screw-axis absolute error: 3.965594685961804e-9
- maximum home grasp TCP transform absolute error: 1.0000333894311098e-12
- visual meshes: 8; collision meshes: 8; manifest components: 8
- Python tests: 24 passed

The mathematical model is therefore internally consistent. This does not prove the physical motor encoder signs, zero offsets, limits, TCP offset, or camera mounting.

## World-model status

The checked-in mesh package converts STEP-world millimetres into URDF link-local metres. It contains eight visual/collision components, but the rigid-body mapping is explicitly marked provisional and requires articulated RViz review. The rendered home pose is useful for spotting gross placement errors; it is not a collision or physical-fit certificate.

## Zero-pose joint centers and axes

All values below are expressed in `base_link`, in metres. Positive rotation follows the right-hand rule around the listed axis.

| Joint | Center (m) | Space axis |
|---|---|---|
| J1 | (0.000000000, 0.000000000, 0.000000000) | (0, 0, 1) |
| J2 | (-0.000000036, 0.000000000, 0.155999957) | (0, 1, 0) |
| J3 | (0.179999574, -0.000000001, 0.156367432) | (0, 1, 0) |
| J4 | (0.189786868, -0.000000001, 0.336101184) | (0, 1, 0) |
| J5 | (0.189786868, 0.092999999, 0.336101186) | (-0.999698994, 0, -0.024534089) |
| J6 | (0.295755048, 0.092999999, 0.338701800) | (0.000416436, 0.999855935, -0.016968653) |

Center-to-center distances computed from the current model:

- J1-J2: 156.000 mm
- J2-J3: 180.000 mm
- J3-J4: 180.000 mm
- J4-J5: 93.000 mm
- J5-J6: 106.000 mm
- J1-J6 center distance: 459.172 mm
- J1-grasp TCP home distance: 466.519 mm
- J1-tip TCP home distance: 475.899 mm

These are model-derived center distances, not encoder zero or usable reach limits. They must be compared with physical measurements before replacing any value.

## Geometry decision

For the offline kinematics, ROS2 description, world model, and simulation, the distances in this table are the canonical values. Earlier conversational dimensions are treated as mixed housing/offset measurements and are not written into the model. A future physical measurement may update the model only when the same joint-axis-center reference is used.

## Source-CAD axis-fit evidence

The original STEP was independently processed by an Open Cascade XCAF axis extractor. Main axes were selected from repeated concentric cylindrical faces, rather than from product reference origins:

- J1 axis fit: 111 cylindrical faces; line RMS 0.000261 mm; max 0.001565 mm.
- J2, J3, J4, J5 and J6 use the corresponding repeated-radius axis families in the STEP assembly.
- The old J2 product origin is `(0,106,0) mm`; the fitted J2 motor axis is at approximately `y=156 mm`. Therefore 106 mm is not the J1-J2 axis-center distance.

Source-derived dimensions (before rounding for the runtime model) are:

- J1 to J2 along J1: `155.999957265 mm`
- J2 to J3 common-axis distance: `179.999984221 mm`
- J3 to J4 common-axis distance: `180.000035182 mm`
- J4 to J5 normal offset: `93.000000000 mm`
- J5 to J6 signed axis distance: `-106.000085963 mm` (magnitude `106.000085963 mm`)
- J6 to grasp TCP: `207.500000 mm`
- J6 to tip TCP: `239.916498 mm`

The source-derived standard-DH rows are:

| Joint | d (m) | a (m) | alpha (deg) | theta_home (deg) |
|---|---:|---:|---:|---:|
| 1 | 0.155999957265 | -0.000000017765 | -90.000000 | 0.000000 |
| 2 | -0.000000000534 | 0.179999966456 | 0.000000 | -0.116971 |
| 3 | 0.000000000000 | 0.180000035182 | 0.000000 | -86.766100 |
| 4 | 0.093000000000 | 0.000000002269 | 90.000000 | -4.522770 |
| 5 | -0.106000085963 | -0.000000001698 | -90.000000 | 0.972572 |
| 6 | -0.207500000000 | 0.000000000000 | 0.000000 | 91.406043 |

Independent source-derived checks are below `4e-11` for DH reconstruction, `5.60e-9` for POE-versus-URDF FK, and `5.61e-9` for POE-versus-DH FK. These are geometry/model consistency results, not proof of encoder zero or real-world calibration.

## Home TCP frame

The current grasp TCP home position is:

`(0.295668637, -0.114470107, 0.342222795) m`.

Its axes expressed in `base_link` are:

- TCP X: `(0.999999913, -0.000416496, 0.000000000)`
- TCP Y: `(0.000007067, 0.016968651, 0.999856022)`
- TCP Z: `(-0.000416436, -0.999855935, 0.016968653)`

The TCP rotation is orthonormal with determinant approximately 1. The production `RPY(pi, 0, yaw)` shortcut is not equivalent to this frame and must not be used as a confirmed grasp orientation.

## Camera/world consistency findings

The checked-in nine-point homography maps the calibration area to approximately:

- X: 259..279 mm
- Y: -240..-220 mm

The runtime workspace gate currently accepts Y only from -180..180 mm. This is a configuration contradiction, not an IK failure. The base origin, camera-to-base transform, XY signs, and table calibration must be remeasured; the gate must not be widened just to make the simulation pass.

There is also configuration drift: `config.demo.env` contains a different demo Y range (-310..-150 mm), while the runtime default is -180..180 mm. These values must be unified only after the physical base/worktable measurement.

## Offline stress result

With the current production workspace gate, 100 noisy trials produced:

- 6 outside the calibrated image region
- 94 outside the configured robot workspace
- 0 accepted for IK

With the workspace gate bypassed for mechanical diagnosis only:

- 97 accepted targets
- current `RPY(pi,0,yaw)` orientation: 0/97 complete sequences
- model-aligned TCP orientation: 95/97 complete sequences (97.94%)

The two remaining model-aligned failures are numerical/limit-boundary cases under the assumed placeholder joint envelope. They do not justify enabling hardware.

## Required physical measurements before changing the model

1. Mark the J1 axis center as the physical base origin and define the real +Z direction.
2. Choose physical +X and derive +Y from the right-hand rule; record the direction relative to the table and camera.
3. Perform passive CAN identification, then one joint at a time at <=5% speed and 1..2 degree commands.
4. Record actual node ID, encoder sign, encoder zero, soft limits, hard limits, feedback rate, and emergency-stop behavior.
5. Measure the flange/TCP origin and the tool approach axis.
6. Rebuild the nine-point homography in the measured base frame and rerun this audit plus the Monte-Carlo tests.

Until these measurements exist, `motion_enabled`, `protocol_confirmed`, `estop_verified`, `feedback_verified`, encoder directions, offsets, and limits remain locked/unmeasured.
