from __future__ import annotations

import contextlib
import io
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PROJECT_DIR / "tools" / "teach_grasp_execute.py"
SPEC = importlib.util.spec_from_file_location("teach_grasp_execute", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
execute = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(execute)


class TeachGraspExecuteTests(unittest.TestCase):
    READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
    TAUGHT_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328,
                  -84.9998664855957, 110.00060272216797, 0.0]

    @staticmethod
    def _effective_limits() -> tuple[list[float], list[float]]:
        config_path = (
            PROJECT_DIR
            / "robot_ai"
            / "arm_control"
            / "config"
            / "hardware_calibration_candidate_20260811.json"
        )
        return execute.limits_deg_from_config(json.loads(config_path.read_text(encoding="utf-8")))

    @staticmethod
    def _compact_stage_plan() -> dict:
        return {
            "duration_s": 0.001,
            "goal_deg": [0.0] * 6,
            "points": [{"positions_deg": [0.0] * 6, "duration_ms": 1}],
        }

    @classmethod
    def _two_stage_route(cls) -> dict:
        return {
            "schema": "xiaou_taught_side_route_execute_plan_v1",
            "stages": [
                {
                    "route_index": 1,
                    "name": "raise_to_transit",
                    "phase": "raise",
                    "execution_plan": cls._compact_stage_plan(),
                },
                {
                    "route_index": 2,
                    "name": "transfer_above_target",
                    "phase": "transfer",
                    "execution_plan": cls._compact_stage_plan(),
                },
            ],
        }

    @staticmethod
    def _settled_snapshot(sequence: int, **_) -> tuple[list[dict], int]:
        return (
            [
                {"joint_id": joint_id, "angle_deg": 0.0, "speed_rpm": 0.0, "online": True}
                for joint_id in range(1, 7)
            ],
            (sequence + 6) & 0xFF,
        )

    def test_ready_to_teach_plan_fits_fifo_and_respects_limits(self) -> None:
        start = [-0.03, -49.96, -55.0, -70.0, 110.0, 0.01]
        goal = [-100.000244, -48.930473, -71.999702, -84.999866, 110.000603, 0.0]
        plan = execute.build_segmented_quintic_plan(
            start,
            goal,
            speed_limit_deg_s=[2.0] * 6,
            accel_limit_deg_s2=[1.0] * 6,
            max_point_duration_ms=9000,
        )
        self.assertLessEqual(plan["segment_count"], 15)
        self.assertLessEqual(plan["segment_duration_ms"], 9000)
        self.assertEqual(plan["points"][-1]["positions_deg"], goal)
        self.assertTrue(all(value <= 2.0 + 1e-9 for value in plan["peak_speed_deg_s"]))
        self.assertTrue(all(value <= 1.0 + 1e-9 for value in plan["peak_accel_deg_s2"]))
        self.assertEqual(sum(len(batch) for batch in execute.partition_batches(plan["points"])), plan["segment_count"])

    def test_segment_acceleration_is_recomputed_for_stop_start_boundaries(self) -> None:
        plan = execute.build_segmented_quintic_plan(
            [0.0] * 6,
            [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed_limit_deg_s=[100.0] * 6,
            accel_limit_deg_s2=[1.0] * 6,
            max_point_duration_ms=1000,
        )
        self.assertGreater(plan["duration_s"], 1.0)
        self.assertLessEqual(plan["peak_accel_deg_s2"][0], 1.0 + 1e-9)

    def test_limit_validation_uses_effective_config_envelope(self) -> None:
        lower = [-165.0, -125.0, -135.0, -175.0, -85.0, -175.0]
        upper = [165.0, 125.0, 135.0, 175.0, 115.0, 175.0]
        execute.validate_pose_limits("goal", [-100.0, -48.93, -72.0, -85.0, 110.0, 0.0], lower, upper)
        with self.assertRaisesRegex(execute.TeachExecutionError, "J2"):
            execute.validate_pose_limits("goal", [0.0, 126.0, -72.0, -85.0, 110.0, 0.0], lower, upper)

    def test_joint_snapshot_requires_six_online_and_stationary(self) -> None:
        snapshot = [
            {"joint_id": joint_id, "angle_deg": float(joint_id), "speed_rpm": 0.1, "online": True}
            for joint_id in range(1, 7)
        ]
        self.assertEqual(
            execute.validate_joint_snapshot(snapshot, require_idle=True, max_abs_speed_rpm=2.0),
            [1, 2, 3, 4, 5, 6],
        )
        snapshot[5]["online"] = False
        with self.assertRaisesRegex(execute.TeachExecutionError, "J6"):
            execute.validate_joint_snapshot(snapshot, require_idle=True, max_abs_speed_rpm=2.0)
        snapshot[5]["online"] = True
        snapshot[0]["speed_rpm"] = 2.1
        with self.assertRaisesRegex(execute.TeachExecutionError, "stationary"):
            execute.validate_joint_snapshot(snapshot, require_idle=True, max_abs_speed_rpm=2.0)

    def test_assumed_joint_provenance_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "pose.json"
            path.write_text(
                '{"schema":"xiaou_teach_pose_v1","read_only":true,'
                '"pose_deg":[0,0,0,0,0,0],"assumed_joint_ids":[6]}',
                encoding="utf-8",
            )
            pose, assumed = execute.load_teach_pose(path)
        self.assertEqual(pose, [0.0] * 6)
        self.assertEqual(assumed, [6])

    def test_more_than_fifo_capacity_is_rejected_before_uart(self) -> None:
        points = [{"positions_deg": [0.0] * 6, "duration_ms": 1000}] * 16
        with self.assertRaisesRegex(execute.TeachExecutionError, "at most 15"):
            execute.partition_batches(points)

    def test_ready_to_teach_batches_are_nine_points_or_fewer(self) -> None:
        plan = execute.build_segmented_quintic_plan(
            [-0.03, -49.96, -55.0, -70.0, 110.0, 0.01],
            [-100.000244, -48.930473, -71.999702, -84.999866, 110.000603, 0.0],
            speed_limit_deg_s=[2.0] * 6,
            accel_limit_deg_s2=[1.0] * 6,
            max_point_duration_ms=9000,
        )
        batches = execute.partition_batches(plan["points"])
        self.assertTrue(all(1 <= len(batch) <= 9 for batch in batches))
        self.assertLessEqual(sum(len(batch) for batch in batches), 15)

    def test_taught_side_route_preserves_cartesian_phase_order_and_joint_limits(self) -> None:
        lower, upper = self._effective_limits()
        route = execute.build_taught_side_route_plan(
            self.READY_DEG,
            self.TAUGHT_DEG,
            lower_deg=lower,
            upper_deg=upper,
            speed_limit_deg_s=[20.0] * 6,
            accel_limit_deg_s2=[12.0] * 6,
            max_point_duration_ms=10_000,
        )
        stages = route["stages"]
        self.assertEqual(route["schema"], "xiaou_taught_side_route_execute_plan_v1")
        self.assertEqual(route["route_stage_count"], 14)
        self.assertEqual(
            [stage["phase"] for stage in stages[:3]],
            ["raise", "transfer", "transfer"],
        )
        self.assertEqual(
            [stage["phase"] for stage in stages[3:9]],
            ["descend"] * 6,
        )
        self.assertEqual(
            [stage["phase"] for stage in stages[9:13]],
            ["side_approach"] * 4,
        )
        self.assertEqual(stages[-1]["name"], "taught_side_contact")
        self.assertEqual(stages[-1]["phase"], "approach_contact")
        self.assertTrue(route["requires_j6_motion"])
        self.assertGreater(route["max_j6_travel_deg"], 0.25)
        for stage in stages:
            target = stage["target_joint_deg"]
            self.assertTrue(all(low - 1e-9 <= value <= high + 1e-9 for value, low, high in zip(target, lower, upper)))
            point_plan = stage["execution_plan"]
            self.assertLessEqual(point_plan["segment_count"], execute.F407_FIFO_CAPACITY - 1)
            self.assertEqual(sum(len(batch) for batch in execute.partition_batches(point_plan["points"])), point_plan["segment_count"])

    def test_local_bottle_can_reuse_route_only_with_measured_dimensions_inside_envelope(self) -> None:
        lower, upper = self._effective_limits()
        route = execute.build_taught_side_route_plan(
            self.READY_DEG,
            self.TAUGHT_DEG,
            lower_deg=lower,
            upper_deg=upper,
            speed_limit_deg_s=[20.0] * 6,
            accel_limit_deg_s2=[12.0] * 6,
            max_point_duration_ms=10_000,
            object_class="bottle",
            object_radius_m=0.033,
            object_height_m=0.192,
            target_offset_base_m=(0.001, 0.0, 0.0),
        )
        self.assertEqual(route["object_class"], "bottle")
        self.assertEqual(route["grasp_family_preview"]["status"], "preview_ready")
        self.assertEqual(route["route_stage_count"], 14)

    def test_nonlocal_or_non_cylinder_object_cannot_reuse_cola_route(self) -> None:
        lower, upper = self._effective_limits()
        shared = {
            "lower_deg": lower,
            "upper_deg": upper,
            "speed_limit_deg_s": [20.0] * 6,
            "accel_limit_deg_s2": [12.0] * 6,
            "max_point_duration_ms": 10_000,
        }
        with self.assertRaisesRegex(execute.TeachExecutionError, "does not define this grasp family"):
            execute.build_taught_side_route_plan(
                self.READY_DEG,
                self.TAUGHT_DEG,
                object_class="cup",
                **shared,
            )
        with self.assertRaisesRegex(execute.TeachExecutionError, "dimensions exceed"):
            execute.build_taught_side_route_plan(
                self.READY_DEG,
                self.TAUGHT_DEG,
                object_class="bottle",
                object_radius_m=0.045,
                object_height_m=0.24,
                **shared,
            )

    def test_taught_side_route_requires_explicit_j6_confirmation_before_uart_execution(self) -> None:
        config_path = (
            PROJECT_DIR
            / "robot_ai"
            / "arm_control"
            / "config"
            / "hardware_calibration_candidate_20260811.json"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pose_path = root / "measured_pose.json"
            plan_path = root / "route.json"
            pose_path.write_text(
                json.dumps(
                    {
                        "schema": "xiaou_teach_pose_v1",
                        "read_only": True,
                        "pose_deg": self.TAUGHT_DEG,
                        "assumed_joint_ids": [],
                    }
                ),
                encoding="utf-8",
            )
            arguments = [
                "teach_grasp_execute.py",
                "--teach-pose", str(pose_path),
                "--hardware-config", str(config_path),
                "--plan-output", str(plan_path),
                "--execute",
                "--confirm", execute.TAUGHT_SIDE_ROUTE_CONFIRM_TOKEN,
                "--firmware-confirmed",
                "--estop-confirmed",
                "--mcu-calibration-confirmed",
            ]
            stderr = io.StringIO()
            with (
                patch.object(execute, "capture_preflight_start", return_value=(self.READY_DEG, 9, [])),
                patch.object(execute, "execute_taught_side_route") as route_executor,
                patch.object(sys, "argv", arguments),
                contextlib.redirect_stderr(stderr),
            ):
                self.assertEqual(execute.main(), 2)
        self.assertIn("allow-route-j6-motion", stderr.getvalue())
        route_executor.assert_not_called()

    def test_joint_preflight_authorization_requires_explicit_firmware_estop_and_calibration(self) -> None:
        base = {
            "motion_enabled": False,
            "protocol_confirmed": True,
            "uart_link_verified": True,
            "f407_firmware_verified": True,
            "estop_verified": True,
            "feedback_verified": False,
            "mcu_calibration_verified": False,
            "protocol_family": "DaRanRobot_F407_UART_V1",
            "transport": {
                "kind": "uart",
                "port": "/dev/serial0",
                "baud": 115200,
                "data_bits": 8,
                "stop_bits": 1,
                "parity": "none",
                "flow_control": False,
                "direct_passthrough": True,
                "protocol_version": "DaRanRobot_F407_UART_V1",
            },
            "joint_node_ids": [1, 2, 3, 4, 5, 6],
            "position_min_rad": [-1.0] * 6,
            "position_max_rad": [1.0] * 6,
            "velocity_max_rad_s": [0.1] * 6,
            "acceleration_max_rad_s2": [0.2] * 6,
            "mcu_calibration": {"authority": "stm32_f407"},
        }
        authorized = execute.build_live_authorized_config(
            base,
            explicit_session_authorized=True,
            live_uart_verified=True,
            live_feedback_verified=True,
            operator_firmware_verified=True,
            operator_estop_verified=True,
            operator_mcu_calibration_verified=True,
        )
        self.assertFalse(base["motion_enabled"])
        self.assertFalse(base["feedback_verified"])
        self.assertTrue(authorized["motion_enabled"])
        self.assertTrue(authorized["feedback_verified"])
        self.assertTrue(authorized["mcu_calibration_verified"])
        self.assertEqual(
            authorized["supervised_session_authorization"]["operator_confirmed_fields"],
            ["f407_firmware_verified", "estop_verified", "mcu_calibration_verified"],
        )
        with self.assertRaisesRegex(execute.TeachExecutionError, "mcu_calibration_verified"):
            execute.build_live_authorized_config(
                base,
                explicit_session_authorized=True,
                live_uart_verified=True,
                live_feedback_verified=True,
                operator_firmware_verified=True,
                operator_estop_verified=True,
                operator_mcu_calibration_verified=False,
            )
        staged = dict(base)
        staged["f407_firmware_verified"] = False
        with self.assertRaisesRegex(execute.TeachExecutionError, "f407_firmware_verified"):
            execute.build_live_authorized_config(
                staged,
                explicit_session_authorized=True,
                live_uart_verified=True,
                live_feedback_verified=True,
                operator_firmware_verified=False,
                operator_estop_verified=True,
                operator_mcu_calibration_verified=True,
            )

    def test_refresh_session_calibration_heartbeat_keeps_temporary_gate_live(self) -> None:
        base = {
            "motion_enabled": False,
            "protocol_confirmed": True,
            "uart_link_verified": True,
            "f407_firmware_verified": True,
            "estop_verified": True,
            "feedback_verified": False,
            "mcu_calibration_verified": False,
            "protocol_family": "DaRanRobot_F407_UART_V1",
            "transport": {
                "kind": "uart", "port": "/dev/serial0", "baud": 115200,
                "data_bits": 8, "stop_bits": 1, "parity": "none",
                "flow_control": False, "direct_passthrough": True,
                "protocol_version": "DaRanRobot_F407_UART_V1",
            },
            "joint_node_ids": [1, 2, 3, 4, 5, 6],
            "position_min_rad": [-1.0] * 6,
            "position_max_rad": [1.0] * 6,
            "velocity_max_rad_s": [0.1] * 6,
            "acceleration_max_rad_s2": [0.2] * 6,
            "mcu_calibration": {"authority": "stm32_f407"},
        }
        authorized = execute.build_live_authorized_config(
            base,
            explicit_session_authorized=True,
            live_uart_verified=True,
            live_feedback_verified=True,
            operator_firmware_verified=True,
            operator_estop_verified=True,
            operator_mcu_calibration_verified=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "live_gate.json"
            path.write_text(__import__("json").dumps(authorized), encoding="utf-8")
            execute.refresh_session_calibration_heartbeat(path)
            refreshed = __import__("json").loads(path.read_text(encoding="utf-8"))
        self.assertEqual(refreshed["mcu_calibration"]["zero_valid"], [True] * 6)
        self.assertLess(abs(time.time() - refreshed["mcu_calibration"]["verified_unix_s"]), 2.0)

    def test_joint_snapshot_uses_compact_joint_replies_without_get_state(self) -> None:
        commands: list[tuple[int, int, bytes]] = []
        original_exchange = execute.exchange
        try:
            def fake_exchange(frame, **_):
                commands.append((frame.cmd, frame.seq, frame.payload))
                joint_id = frame.payload[0]
                payload = bytes((joint_id,)) + struct.pack("<fffB", float(joint_id), 0.0, 0.0, 1)
                return execute.Frame(execute.RSP_ACK, frame.seq, payload)

            execute.exchange = fake_exchange
            snapshot, next_sequence = execute._read_joint_snapshot(
                11, port="unused", baud=115200, timeout_s=1.0, attempts=1
            )
        finally:
            execute.exchange = original_exchange
        self.assertEqual([item["joint_id"] for item in snapshot], [1, 2, 3, 4, 5, 6])
        self.assertEqual(next_sequence, 17)
        self.assertEqual([item[0] for item in commands], [execute.CMD_GET_JOINT] * 6)
        self.assertEqual([item[2] for item in commands], [bytes((joint_id,)) for joint_id in range(1, 7)])

    def test_uncertain_point_ack_sends_stop_without_retrying_motion(self) -> None:
        commands: list[tuple[int, int]] = []
        original_exchange = execute.exchange
        original_refresh = execute.refresh_session_calibration_heartbeat
        try:
            def fake_exchange(frame, **_):
                commands.append((frame.cmd, frame.seq))
                if frame.cmd == execute.CMD_TRAJ_BUFFER_CLEAR:
                    return execute.Frame(execute.RSP_TRAJ_ACK, frame.seq, bytes((16,)))
                if frame.cmd == execute.CMD_TRAJ_POINT:
                    raise TimeoutError("unknown admission result")
                if frame.cmd == execute.CMD_STOP:
                    return execute.Frame(execute.RSP_ACK, frame.seq)
                raise AssertionError(f"unexpected command {frame.cmd}")

            execute.exchange = fake_exchange
            execute.refresh_session_calibration_heartbeat = lambda _path: None
            plan = {
                "duration_s": 1.0,
                "goal_deg": [0.0] * 6,
                "points": [{"positions_deg": [0.0] * 6, "duration_ms": 1000}],
            }
            with self.assertRaisesRegex(execute.TeachExecutionError, "STOP acknowledged"):
                execute.execute_plan(
                    plan,
                    hardware_config_path=Path("unused.json"),
                    port="unused",
                    baud=115200,
                    timeout_s=1.0,
                    poll_interval_s=0.01,
                    completion_margin_s=1.0,
                    completion_tolerance_deg=0.5,
                    sequence=1,
                )
        finally:
            execute.exchange = original_exchange
            execute.refresh_session_calibration_heartbeat = original_refresh
        self.assertEqual(
            commands,
            [
                (execute.CMD_TRAJ_BUFFER_CLEAR, 1),
                (execute.CMD_TRAJ_POINT, 2),
                (execute.CMD_STOP, 3),
            ],
        )

    def test_clear_contract_requires_trajectory_ack_with_full_fifo(self) -> None:
        commands: list[tuple[int, int]] = []
        original_exchange = execute.exchange
        original_refresh = execute.refresh_session_calibration_heartbeat
        try:
            def fake_exchange(frame, **_):
                commands.append((frame.cmd, frame.seq))
                if frame.cmd == execute.CMD_TRAJ_BUFFER_CLEAR:
                    return execute.Frame(execute.RSP_ACK, frame.seq)
                if frame.cmd == execute.CMD_STOP:
                    return execute.Frame(execute.RSP_ACK, frame.seq)
                raise AssertionError(f"unexpected command {frame.cmd}")

            execute.exchange = fake_exchange
            execute.refresh_session_calibration_heartbeat = lambda _path: None
            plan = {
                "duration_s": 1.0,
                "goal_deg": [0.0] * 6,
                "points": [{"positions_deg": [0.0] * 6, "duration_ms": 1000}],
            }
            with self.assertRaisesRegex(execute.TeachExecutionError, "trajectory-clear ACK.*STOP acknowledged"):
                execute.execute_plan(
                    plan,
                    hardware_config_path=Path("unused.json"),
                    port="unused",
                    baud=115200,
                    timeout_s=1.0,
                    poll_interval_s=0.01,
                    completion_margin_s=1.0,
                    completion_tolerance_deg=0.5,
                    sequence=7,
                )
        finally:
            execute.exchange = original_exchange
            execute.refresh_session_calibration_heartbeat = original_refresh
        self.assertEqual(
            commands,
            [
                (execute.CMD_TRAJ_BUFFER_CLEAR, 7),
                (execute.CMD_STOP, 8),
            ],
        )

    def test_compact_points_wait_for_settle_before_next_admission(self) -> None:
        commands: list[tuple[int, int]] = []
        original_exchange = execute.exchange
        original_read_joint_snapshot = execute._read_joint_snapshot
        original_refresh = execute.refresh_session_calibration_heartbeat
        try:
            def fake_exchange(frame, **_):
                commands.append((frame.cmd, frame.seq))
                if frame.cmd == execute.CMD_TRAJ_BUFFER_CLEAR:
                    return execute.Frame(execute.RSP_TRAJ_ACK, frame.seq, bytes((16,)))
                if frame.cmd == execute.CMD_TRAJ_POINT:
                    free_slots = 16 - sum(command == execute.CMD_TRAJ_POINT for command, _ in commands)
                    return execute.Frame(execute.RSP_TRAJ_ACK, frame.seq, bytes((free_slots,)))
                raise AssertionError(f"unexpected command {frame.cmd}")

            def fake_read_joint_snapshot(sequence, **_):
                return ([
                    {"joint_id": joint_id, "angle_deg": 0.0, "speed_rpm": 0.0, "online": True}
                    for joint_id in range(1, 7)
                ], (sequence + 6) & 0xFF)

            execute.exchange = fake_exchange
            execute._read_joint_snapshot = fake_read_joint_snapshot
            execute.refresh_session_calibration_heartbeat = lambda _path: None
            result = execute.execute_plan(
                {
                    "duration_s": 0.1,
                    "goal_deg": [0.0] * 6,
                    "points": [
                        {"positions_deg": [0.0] * 6, "duration_ms": 1000},
                        {"positions_deg": [0.0] * 6, "duration_ms": 1000},
                    ],
                },
                hardware_config_path=Path("unused.json"),
                port="unused",
                baud=115200,
                timeout_s=1.0,
                poll_interval_s=0.001,
                completion_margin_s=1.0,
                completion_tolerance_deg=0.5,
                sequence=31,
                segment_quiet_settle_s=0.0,
            )
        finally:
            execute.exchange = original_exchange
            execute._read_joint_snapshot = original_read_joint_snapshot
            execute.refresh_session_calibration_heartbeat = original_refresh
        self.assertTrue(result["completed"])
        self.assertEqual(
            commands,
            [
                (execute.CMD_TRAJ_BUFFER_CLEAR, 31),
                (execute.CMD_TRAJ_POINT, 32),
                # Two six-axis settled snapshots are required before point 2.
                (execute.CMD_TRAJ_POINT, 45),
            ],
        )

    def test_taught_side_route_clears_admits_and_settles_each_stage_in_order(self) -> None:
        commands: list[tuple[int, int]] = []
        original_exchange = execute.exchange
        original_read_joint_snapshot = execute._read_joint_snapshot
        original_refresh = execute.refresh_session_calibration_heartbeat
        try:
            def fake_exchange(frame, **_):
                commands.append((frame.cmd, frame.seq))
                if frame.cmd == execute.CMD_TRAJ_BUFFER_CLEAR:
                    return execute.Frame(execute.RSP_TRAJ_ACK, frame.seq, bytes((16,)))
                if frame.cmd == execute.CMD_TRAJ_POINT:
                    return execute.Frame(execute.RSP_TRAJ_ACK, frame.seq, bytes((15,)))
                raise AssertionError(f"unexpected command {frame.cmd}")

            execute.exchange = fake_exchange
            execute._read_joint_snapshot = self._settled_snapshot
            execute.refresh_session_calibration_heartbeat = lambda _path: None
            result = execute.execute_taught_side_route(
                self._two_stage_route(),
                hardware_config_path=Path("unused.json"),
                port="unused",
                baud=115200,
                timeout_s=1.0,
                poll_interval_s=0.001,
                completion_margin_s=0.01,
                completion_tolerance_deg=0.5,
                sequence=21,
                active_feedback_timeout_s=0.5,
                active_feedback_attempts=1,
                segment_quiet_settle_s=0.0,
                segment_settled_samples=1,
            )
        finally:
            execute.exchange = original_exchange
            execute._read_joint_snapshot = original_read_joint_snapshot
            execute.refresh_session_calibration_heartbeat = original_refresh
        self.assertTrue(result["completed"])
        self.assertEqual(result["execution_mode"], "taught_side_route_stage_by_stage")
        self.assertEqual(result["route_stages_completed"], 2)
        self.assertEqual([stage["name"] for stage in result["stages"]], ["raise_to_transit", "transfer_above_target"])
        self.assertEqual(
            commands,
            [
                (execute.CMD_TRAJ_BUFFER_CLEAR, 21),
                (execute.CMD_TRAJ_POINT, 22),
                (execute.CMD_TRAJ_BUFFER_CLEAR, 29),
                (execute.CMD_TRAJ_POINT, 30),
            ],
        )

    def test_taught_side_route_stage_failure_is_labeled_and_does_not_admit_later_stages(self) -> None:
        commands: list[tuple[int, int]] = []
        original_execute_plan = execute.execute_plan
        try:
            def fake_execute_plan(plan, **kwargs):
                commands.append((int(plan["route_marker"]), int(kwargs["sequence"])))
                if plan["route_marker"] == 1:
                    return {
                        "segments_completed": 1,
                        "final_deg": [0.0] * 6,
                        "max_error_deg": 0.0,
                        "next_sequence": 44,
                    }
                raise execute.TeachExecutionError("matching ACK was lost; STOP acknowledged", next_sequence=46)

            route = self._two_stage_route()
            route["stages"][0]["execution_plan"]["route_marker"] = 1
            route["stages"][1]["execution_plan"]["route_marker"] = 2
            execute.execute_plan = fake_execute_plan
            with self.assertRaisesRegex(execute.TeachExecutionError, r"stage 2 \(transfer_above_target\).*STOP acknowledged"):
                execute.execute_taught_side_route(
                    route,
                    hardware_config_path=Path("unused.json"),
                    port="unused",
                    baud=115200,
                    timeout_s=1.0,
                    poll_interval_s=0.01,
                    completion_margin_s=1.0,
                    completion_tolerance_deg=0.5,
                    sequence=31,
                )
        finally:
            execute.execute_plan = original_execute_plan
        self.assertEqual(commands, [(1, 31), (2, 44)])


if __name__ == "__main__":
    unittest.main()
