# XiaoU six-axis mechanical model parameter package

Date: 2026-08-06
Source: user-supplied total assembly STEP, extracted with Open Cascade XCAF.
Scope: geometry and offline model only. No real CAN or actuator motion was used.

## Authority and units

The source-derived parameter file is `D:\机械臂\robot_model_parameters.json`. The runtime POE/TCP values in `robot_ai/arm_control/config/arm_model.json` match that source file exactly for all screw-axis and TCP matrix entries. Geometry is reported in millimetres below; ROS2/POE uses metres and radians.

The STEP assembly is the home pose, defined as `q1..q6 = 0`. Positive mathematical joint motion follows the right-hand rule around the listed axis. This is not yet the encoder sign.

## Base frame

`base_link` is defined from the CAD axes:

- origin in STEP world: `(-0.000007933, 0, 0.000042735) mm`
- +Z: `(0, 1, 0)` in STEP world, the J1 axis
- +Y: `(0.980886458, 0, 0.194580977)` in STEP world, the J2 axis at home
- +X: `(-0.194580977, 0, 0.980886458)` in STEP world, completing a right-handed frame

In `base_link`, +Z is J1, +Y is the shoulder/elbow axis family, and +X completes the right-handed system.

## Joint axes and home origins

| Joint | Origin in base (mm) | Axis in base |
|---|---|---|
| J1 | `(0, 0, 0)` | `(0, 0, 1)` |
| J2 | `(-0.000018, 0, 155.999957)` | `(0, 1, 0)` |
| J3 | `(179.999574, -0.000001, 156.367432)` | `(0, 1, 0)` |
| J4 | `(189.786868, -0.000001, 336.101184)` | `(0, 1, 0)` |
| J5 | `(189.786868, 93.000000, 336.101186)` | `(-0.999698994, 0, -0.024534089)` |
| J6 | `(295.755048, 92.999999, 338.701800)` | `(0.000416436, 0.999855935, -0.016968653)` |

The source CAD fit uses repeated concentric cylindrical faces. For J1 the fit used 111 faces with 0.000261 mm RMS line error and 0.001565 mm maximum line error; the other motor axes use the same repeated-radius method.

## Canonical geometric distances

- J1 to J2 along J1: `155.999957 mm`
- J2 to J3 axis distance: `179.999984 mm`
- J3 to J4 axis distance: `180.000035 mm`
- J4 to J5 normal offset: `93.000000 mm`
- J5 to J6 signed axis distance: `-106.000086 mm`; magnitude `106.000086 mm`
- J1 to J6 centre straight distance in the home pose: `459.172035 mm`

The old J2 product reference origin at `y=106 mm` is not the J2 motor axis. It must not be used as the J1-J2 link length.

## Standard DH parameters

Convention: `A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`, with `theta_i = theta_home_i + q_i`.

| Joint | d (m) | a (m) | alpha (deg) | theta_home (deg) |
|---|---:|---:|---:|---:|
| 1 | 0.155999957265 | -0.000000017765 | -90.000000 | 0.000000 |
| 2 | -0.000000000534 | 0.179999966456 | 0.000000 | -0.116971 |
| 3 | 0.000000000000 | 0.180000035182 | 0.000000 | -86.766100 |
| 4 | 0.093000000000 | 0.000000002269 | 90.000000 | -4.522770 |
| 5 | -0.106000085963 | -0.000000001698 | -90.000000 | 0.972572 |
| 6 | -0.207500000000 | 0.000000000000 | 0.000000 | 91.406043 |

Source checks: DH row reconstruction `3.85e-11`, POE/URDF FK `5.60e-9`, POE/DH FK `5.61e-9`.

## TCP and end effector

The current tool direction is the negative J6 axis.

### Grasp TCP

- definition: midpoint of the longest distal linear inner-face contact segment
- distance from J6 axis centre: `207.500000 mm`
- home position in base: `(295.668637, -114.470107, 342.222795) mm`
- contact segment axial range: `177.5..237.5 mm`
- predicted open inner diameter at TCP: `67.320328 mm`
- linear-fit maximum residual: `0.033366 mm`
- geometric confidence: high; functional contact calibration: medium

### Tip TCP

- definition: mean distal extent of the three finger meshes
- distance from J6 axis centre: `239.916498 mm`
- home position in base: `(295.655138, -146.881935, 342.772860) mm`
- three tip axial values spread by only `0.000071 mm`

The grasp TCP and tip TCP share the same orientation matrix. Do not replace this frame with an unverified `RPY(pi,0,yaw)` shortcut.

## J6 flange and mounting interface

The PDA-06-36 CAD component contains these repeated flange patterns:

- three-hole PCD: `36 mm`, pitch radius `18 mm`
- three-hole PCD: `48 mm`, pitch radius `24 mm`
- plus-axis flange outer diameter: `54.0 mm`
- minus-axis flange outer diameter: `53.6 mm`

The tool-side separate boss is different from the patterned flange face:

- boss diameter: `17.0 mm` (radius `8.5 mm`)
- tool-side outer face diameter: `55.0 mm` (radius `27.5 mm`)
- no three-hole pattern was detected on that tool-side face
- no central circular bore was proven by the CAD extraction

Therefore the gripper connector must be checked against the boss-side interface; do not assume the PCD36/PCD48 face is the gripper mating face.

## Geometry-derived inertial estimates

The CAD evidence contains closed volumes and inertia normalized per assigned mass. These are useful starting values only; they are not final dynamics parameters because material density, motors, fasteners, wires and overlapping bodies are not calibrated.

| Link | Volume (mm^3) | Uniform-density COM in base (mm) | Ixx/Iyy/Izz per kg (kg m^2) |
|---|---:|---|---|
| link_1 | 534337.846 | `(0.001, -0.010, 39.477)` | `0.001706, 0.001716, 0.000915` |
| link_2 | 357387.959 | `(0.015, -1.706, 148.612)` | `0.001425, 0.001022, 0.001290` |
| link_3 | 727641.187 | `(97.308, 63.332, 156.200)` | `0.000925, 0.006147, 0.006144` |
| link_4 | 727579.966 | `(185.291, -6.330, 253.533)` | `0.006129, 0.006147, 0.000941` |
| link_5 | 357388.522 | `(198.621, 85.614, 336.299)` | `0.001022, 0.001300, 0.001436` |
| link_6 | 357387.670 | `(288.368, 84.165, 338.651)` | `0.001300, 0.001022, 0.001436` |
| connector | 20945.497 | `(295.730, 33.670, 339.709)` | `0.000139, 0.000343, 0.000244` |
| gripper | 485687.514 | `(295.280, -49.415, 343.873)` | `0.001601, 0.000997, 0.001509` |

To obtain mass from volume, the real material and density must be assigned. Do not use these normalized inertias directly in a dynamic controller without that assignment.

## Parameters not determinable from the STEP

The following remain unknown and must be measured or read from the real drive/controller:

- encoder zero offsets and encoder sign for J1..J6
- real CAN node IDs and frame scaling
- positive/negative mechanical and software limits
- maximum velocity, acceleration, torque/current and thermal limits
- backlash, reducer ratio under load, friction and compliance
- actual link and motor masses, centre of mass and complete inertia tensors
- real TCP contact point after gripper closure and payload
- camera-to-base transform, table Z and workspace limits
- collision clearances under cables, screws, wiring and actual fixtures

## ROS2 parameter status

The current ROS2 model may use the CAD geometry for offline FK/IK and planning review. Hardware execution must remain locked until the unmeasured values above are populated. `motion_enabled`, protocol confirmation, E-stop verification, feedback verification, encoder offsets, directions and limits must not be replaced with guessed values.

## Required acceptance sequence

1. Confirm the CAD model in RViz at the STEP home pose and check each visual/collision body articulation.
2. Measure the real base origin and camera/table frame; rebuild the 2D homography.
3. Passively identify CAN nodes and feedback without sending motion commands.
4. Calibrate one joint at a time at low speed and small angle with a reachable E-stop.
5. Populate zero, direction, limits, feedback and TCP contact data.
6. Re-run FK/IK, MoveIt collision checks and noisy pick simulation.
7. Only then consider staged real motion, starting with one unloaded axis.
