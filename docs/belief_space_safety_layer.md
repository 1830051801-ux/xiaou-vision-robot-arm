# Belief-Space Scenario-CVaR Safety Layer

## Why It Exists

The base policy receives one calibrated visual grasp pose and one placement pose. A single plausible-looking pose can still be fragile when the detector has a transient localization error or a false positive. Averaging action chunks does not answer whether the resulting plan remains safe under nearby perception outcomes.

This layer adds an explicit offline decision path between visual geometry and the final six-axis preview:

1. collect repeated calibrated pose observations;
2. build a robust target-pose belief rather than trusting one observation;
3. sample several task-conditioned action chunks from the diffusion policy;
4. project every proposal through bounded multi-start IK and TCP scene review for every observation scenario;
5. choose the candidate with the best safety coverage and lower-tail clearance, or abstain.

It is an engineering safety mechanism for the checked-in POE digital twin. It does not expose any ROS hardware execution, CAN, serial, or real-arm command path.

## Robust Pose Belief

`fuse_pose_belief()` accepts a primary observation plus repeated calibrated pose estimates. It keeps the original non-yaw orientation, estimates base-frame translation and relative yaw, and iteratively applies Huber weights to translation/yaw residuals. The output contains:

- fused SE(3) pose;
- XY, Z, and yaw uncertainty summaries;
- inlier fraction;
- effective weighted observation count;
- maximum normalized residual.

The benchmark helper `synthesize_multiview_observations()` is explicitly offline-only. It creates independent synthetic re-observations around the clean simulated pose and injects configurable outliers. A deployed system must replace this helper with real, independently calibrated pose estimates.

## Scenario-CVaR Candidate Selection

The policy receives the fused pose belief and samples `K` six-axis action chunks. For every candidate and observation scenario, the planner runs:

1. task-profile region validation;
2. multi-start bounded IK for grasp, place, and Cartesian approach poses;
3. task-process reconstruction through approach/contact/transfer/retreat or insertion dwell phases;
4. TCP table and fixture keep-out clearance review.

Only candidates safe in at least the configured scenario fraction are eligible. Eligible candidates are ranked by:

1. scenario-safe fraction;
2. lower-tail CVaR clearance margin (mean clearance across the worst 25% of scenarios);
3. joint travel;
4. mean joint acceleration.

The first two terms dominate. Motion terms are only tie breakers between equally robust paths. The final fused-pose projection is then rechecked before an offline preview is emitted.

## OOD Guarding

The policy uses the fused uncertainty, but the training-support gate deliberately uses the original declared sensor scale. This separation matters: multiple noisy observations can reduce random localization error, yet they must not make an out-of-distribution camera/noise condition appear in-distribution. The high-noise test verifies this behavior.

## Reproducible Evidence

Reference configuration:

| Setting | Value |
| --- | --- |
| Model | `process_graph_multitask_action_chunk_transformer_diffusion` |
| Action chunk | 32 x 6 joint states |
| Tasks | transfer, zone A, zone B, inspection scan, precision insert |
| Balanced nominal stress set | 100 episodes, 20 per task |
| Pose observations | 5 per grasp/place target |
| Synthetic outlier probability | 20% for extra observations |
| Diffusion candidates | 3 |
| Minimum scenario-safe fraction | 75% |

Nominal stress result:

| Metric | Result |
| --- | ---: |
| Primary mean grasp/place pose error | 4.47 mm |
| Robust-belief mean pose error | 2.51 mm |
| Relative reduction | 43.78% |
| Accepted high-confidence plans | 63.00% |
| Mean safe scenario fraction for accepted plans | 93.33% |
| Mean lower-tail CVaR TCP clearance margin | 44.49 mm |
| Mean grasp endpoint error of selected plans | 2.54 mm |

On the balanced high-noise OOD set (XY 8 mm, Z 4 mm, yaw 5 deg), the regular support gate and the belief-space gate both abstained on all 100 episodes. This is intentionally a refusal result, not a success-rate claim.

Run the same nominal stress evaluation:

```powershell
& 'D:\EmbodiedAI\mujoco-venv\Scripts\python.exe' tools\constraint_diffusion_twin.py evaluate `
  --checkpoint runtime\constraint_diffusion\process_graph_multitask_action_chunk_transformer_policy.pt `
  --dataset runtime\constraint_diffusion\multitask_test.npz `
  --output runtime\constraint_diffusion\belief_stress.json `
  --episodes-per-task 20 --samples-per-context 3 --counterfactual-rollouts 4 `
  --belief-views 5 --belief-candidate-count 3 `
  --belief-outlier-probability 0.20 --belief-minimum-safe-fraction 0.75
```

## Boundaries

- Poses, sensor uncertainty, objects, and scene obstacles are synthetic benchmark inputs.
- The scene checker validates TCP clearance only; it is not full-link collision, self-collision, actuator-dynamics, or force-contact validation.
- The method is not a pretrained VLA, raw-RGB policy, real-world multi-camera result, or industrial safety certification.
- Real-arm execution remains locked until measured calibration, encoder feedback, limits, emergency stop, protocol verification, and explicit authorization are available.
