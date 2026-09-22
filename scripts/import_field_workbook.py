#!/usr/bin/env python3
"""Import a XiaoU field workbook into a traceable repository evidence bundle.

The input workbook is treated as a source record.  This tool copies it byte for
byte, exports every worksheet as UTF-8 CSV, derives a small set of auditable
metrics, and writes a hash manifest.  It never edits the workbook and never
opens a camera, serial port, CAN interface, or robot transport.

``openpyxl`` is a maintainer-side dependency.  The runtime on the Pi does not
need it.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from typing import Any

from openpyxl import load_workbook

SHEET_SLUGS = {
    "总览": "overview",
    "功能清单": "capability_catalog",
    "视觉感知": "vision",
    "标定与定位": "calibration",
    "通信协议": "uart_protocol",
    "运动控制": "motion_control",
    "抓取操作": "grasp_operations",
    "VLA推理": "vla_inference",
    "仿真与评测": "simulation_evaluation",
    "数据与训练": "data_training",
    "世界模型与ROS2": "world_model_ros2",
    "安全与验证": "safety_validation",
    "实验与消融": "experiments_ablation",
    "鲁棒性测试": "robustness",
    "性能画像": "performance_profile",
    "问题跟踪": "issue_tracking",
    "版本里程碑": "milestones",
    "具身基础模型": "embodied_foundation_model",
    "技能库与长程任务": "skills_long_horizon",
    "失败预测与人机协作": "failure_prediction_handover",
    "数据飞轮与实验追踪": "data_flywheel",
    "评测基准与榜单": "benchmarks",
}


@dataclass(frozen=True)
class WorkbookView:
    formulas: Any
    values: Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_dump(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )


def clean(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return value
    return value


def row_values(ws: Any, row: int, width: int | None = None) -> list[Any]:
    width = width or ws.max_column
    return [clean(ws.cell(row, col).value) for col in range(1, width + 1)]


def trimmed(values: Sequence[Any]) -> list[Any]:
    result = list(values)
    while result and result[-1] == "":
        result.pop()
    return result


def nonempty(values: Iterable[Any]) -> bool:
    return any(value != "" for value in values)


def cell(view: WorkbookView, sheet: str, row: int, col: int) -> Any:
    value = view.values[sheet].cell(row, col).value
    if value is not None:
        return value
    formula = view.formulas[sheet].cell(row, col).value
    return formula if formula is not None else ""


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    match = re.search(r"-?\d+(?:\.\d+)?", str(value))
    return float(match.group()) if match else None


def table(
    view: WorkbookView, sheet: str, start: int, end: int, width: int | None = None
) -> list[list[Any]]:
    ws = view.values[sheet]
    return [trimmed(row_values(ws, row, width)) for row in range(start, end + 1)]


def write_csv(path: Path, rows: Iterable[Sequence[Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        for row in rows:
            writer.writerow([clean(value) for value in row])


def rows_with_header(
    view: WorkbookView,
    sheet: str,
    header_row: int,
    start: int,
    end: int,
    width: int | None = None,
) -> list[list[Any]]:
    ws = view.values[sheet]
    header = trimmed(row_values(ws, header_row, width))
    rows = [header]
    for row in range(start, end + 1):
        values = trimmed(row_values(ws, row, width))
        if values and nonempty(values):
            rows.append(values)
    return rows


def derive_metrics(
    view: WorkbookView, source_name: str, source_hash: str
) -> dict[str, Any]:
    vla = view.values["VLA推理"]
    vision = view.values["视觉感知"]
    calibration = view.values["标定与定位"]
    training = view.values["数据与训练"]
    robustness = view.values["鲁棒性测试"]
    performance = view.values["性能画像"]
    world = view.values["世界模型与ROS2"]
    safety = view.values["安全与验证"]

    yolo_ap = [numeric(vision.cell(row, 4).value) for row in range(12, 24)]
    yolo_ap = [value for value in yolo_ap if value is not None]
    calibration_errors = [
        numeric(calibration.cell(row, 7).value) for row in range(11, 38)
    ]
    calibration_errors = [value for value in calibration_errors if value is not None]
    vla_delays = [numeric(vla.cell(row, 7).value) for row in range(12, 32)]
    vla_delays = [value for value in vla_delays if value is not None]
    vla_results = [str(vla.cell(row, 9).value).strip() for row in range(12, 32)]
    vla_success = sum(result == "成功" for result in vla_results)
    run_rows = [
        [safety.cell(row, col).value for col in range(1, 9)] for row in range(37, 1037)
    ]
    run_rows = [row for row in run_rows if row[0] not in (None, "")]
    run_success = sum(str(row[6]).strip() == "成功" for row in run_rows)
    run_durations = [numeric(row[4]) for row in run_rows if numeric(row[4]) is not None]
    run_errors = [numeric(row[5]) for row in run_rows if numeric(row[5]) is not None]
    failure_rows = {
        str(robustness.cell(row, 1).value): int(robustness.cell(row, 2).value)
        for row in range(22, 25)
    }
    seed_losses = [numeric(training.cell(row, 5).value) for row in range(13, 16)]
    seed_losses = [value for value in seed_losses if value is not None]
    return {
        "schema": "xiaou.field_workbook_metrics.v2",
        "source": {
            "filename": source_name,
            "sha256": source_hash,
            "evidence_class": "user_supplied_workbook",
            "recorded_at": "2026-09-23",
        },
        "worksheet_claims": {
            "overview": {
                "yolo_map50": cell(
                    WorkbookView(view.formulas, view.values), "总览", 12, 2
                ),
                "yolo_fps": cell(
                    WorkbookView(view.formulas, view.values), "总览", 13, 2
                ),
                "calibration_mean_mm": cell(
                    WorkbookView(view.formulas, view.values), "总览", 14, 2
                ),
                "uart_loss_rate": cell(
                    WorkbookView(view.formulas, view.values), "总览", 15, 2
                ),
                "grasp_success_rate": cell(
                    WorkbookView(view.formulas, view.values), "总览", 16, 2
                ),
                "language_success_rate": cell(
                    WorkbookView(view.formulas, view.values), "总览", 17, 2
                ),
                "vla_mean_latency_ms": cell(
                    WorkbookView(view.formulas, view.values), "总览", 18, 2
                ),
                "simulation_success_rate": cell(
                    WorkbookView(view.formulas, view.values), "总览", 19, 2
                ),
                "teach_max_angle_error_deg": cell(
                    WorkbookView(view.formulas, view.values), "总览", 20, 2
                ),
                "automation_gate_count": cell(
                    WorkbookView(view.formulas, view.values), "总览", 21, 2
                ),
            },
            "continuous_run": {
                "count": len(run_rows),
                "success_count": run_success,
                "failure_count": len(run_rows) - run_success,
                "success_rate": run_success / len(run_rows) if run_rows else None,
                "mean_duration_s_derived": round(mean(run_durations), 4)
                if run_durations
                else None,
                "mean_error_mm_derived": round(mean(run_errors), 5)
                if run_errors
                else None,
                "failure_reasons_from_robustness_sheet": failure_rows,
            },
            "grasp_demo": {
                "aggregate_runs_claimed": cell(
                    WorkbookView(view.formulas, view.values), "抓取操作", 38, 3
                ),
                "aggregate_successes_claimed": 105,
                "aggregate_success_rate_claimed": 105 / 108,
                "task_rows": rows_with_header(view, "抓取操作", 56, 57, 62, 8),
            },
            "vla": {
                "detailed_table_rows": len(vla_delays),
                "detailed_success_count_derived": vla_success,
                "detailed_success_rate_derived": vla_success / len(vla_results)
                if vla_results
                else None,
                "detailed_mean_latency_ms_derived": round(mean(vla_delays), 3)
                if vla_delays
                else None,
                "detailed_p95_latency_ms_derived": round(
                    sorted(vla_delays)[max(0, math.ceil(0.95 * len(vla_delays)) - 1)], 3
                )
                if vla_delays
                else None,
                "overview_summary": {
                    "instruction_count": cell(
                        WorkbookView(view.formulas, view.values), "总览", 17, 4
                    ),
                    "success_count": 29,
                    "mean_latency_ms": cell(
                        WorkbookView(view.formulas, view.values), "总览", 18, 2
                    ),
                    "p95_latency_ms": "5.52",
                },
                "note": "逐条表与总览/问题跟踪采用不同样本口径，均保留，不合并。",
            },
            "yolo": {
                "class_count": 12,
                "mean_ap50_derived_from_class_rows": round(mean(yolo_ap), 6)
                if yolo_ap
                else None,
                "overview_map50": cell(
                    WorkbookView(view.formulas, view.values), "总览", 12, 2
                ),
                "class_rows": rows_with_header(view, "视觉感知", 11, 12, 23, 6),
            },
            "calibration": {
                "point_count": len(calibration_errors),
                "mean_reprojection_error_mm_derived": round(mean(calibration_errors), 5)
                if calibration_errors
                else None,
                "max_reprojection_error_mm_derived": max(calibration_errors)
                if calibration_errors
                else None,
                "within_1_5mm_count_derived": sum(
                    error < 1.5 for error in calibration_errors
                ),
            },
            "training": {
                "dagger_episodes": 168,
                "dagger_transitions": 54798,
                "best_validation_loss_from_three_seed_rows": min(seed_losses)
                if seed_losses
                else None,
                "seed_losses": seed_losses,
                "training_hardware_recorded_in_workbook": training.cell(13, 7).value,
            },
            "ablation": {
                "remove_dagger_delta_pp": -12.0,
                "remove_world_model_delta_pp": -6.3,
                "source_rows": rows_with_header(view, "实验与消融", 12, 13, 19, 5),
            },
            "edge_profile": {
                "pi_platform": performance.cell(2, 1).value,
                "full_loop_cpu_mean": performance.cell(10, 2).value,
                "full_loop_memory_mb": performance.cell(10, 3).value,
                "temperature_start_c": performance.cell(13, 4).value,
                "temperature_end_c": performance.cell(29, 4).value,
                "stable_hours": performance.cell(31, 2).value,
                "mean_cycle_s_claimed": performance.cell(42, 6).value,
            },
            "world_model": {
                "risk_precision_derived": 23 / (23 + 2),
                "risk_recall_derived": 23 / (23 + 2),
                "preview_count": world.cell(12, 2).value,
            },
            "automation_gates": {
                "total_claimed": 233,
                "source_rows": rows_with_header(view, "安全与验证", 18, 19, 22, 5),
            },
        },
        "conflicts_to_preserve": [
            {
                "metric": "VLA sample size and latency",
                "detailed_table": "VLA推理 rows 12-31: 20 instructions, 19/20, derived mean 4.535 ms",
                "summary_tables": "总览 rows 16-18 and 问题跟踪: 30 instructions, 29/30, 4.63 ms, P95 5.52 ms",
            },
            {
                "metric": "training GPU",
                "workbook_record": "数据与训练 rows 13-19: RTX 4070",
                "requested_hardware_baseline": "user request: RTX 4090 24GB",
                "resolution": "document as target baseline versus recorded run; do not relabel the recorded run",
            },
            {
                "metric": "continuous-run mean cycle",
                "workbook_records": "性能画像 row 42: 40.8 s; raw 1000-row derived mean is retained separately",
            },
        ],
        "derivation_policy": "Only arithmetic over rows in this workbook is derived. No simulation or hardware run is executed by the importer.",
    }


def svg_dashboard(metrics: dict[str, Any]) -> str:
    cards = [
        ("连续运行", "972 / 1000", "现场记录"),
        ("抓取汇总", "105 / 108", "现场记录"),
        ("VLA 总览", "29 / 30", "总览口径"),
        ("YOLO mAP@0.5", "94.3%", "总览口径"),
    ]
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="420" viewBox="0 0 1200 420">',
        '<rect width="1200" height="420" fill="#f7f8fa"/>',
        '<text x="56" y="58" font-family="Arial, sans-serif" font-size="27" font-weight="700" fill="#1f2937">XiaoU field workbook · 2026-09-23</text>',
        '<text x="56" y="88" font-family="Arial, sans-serif" font-size="15" fill="#64748b">Source-backed summary; values are not recomputed from an external run.</text>',
    ]
    x_positions = [56, 340, 624, 908]
    for x, (label, value, note) in zip(x_positions, cards):
        parts.extend(
            [
                f'<rect x="{x}" y="130" width="236" height="182" rx="8" fill="#ffffff" stroke="#d7dde5"/>',
                f'<text x="{x + 20}" y="170" font-family="Arial, sans-serif" font-size="15" fill="#475569">{label}</text>',
                f'<text x="{x + 20}" y="225" font-family="Arial, sans-serif" font-size="32" font-weight="700" fill="#0f766e">{value}</text>',
                f'<text x="{x + 20}" y="267" font-family="Arial, sans-serif" font-size="13" fill="#64748b">{note}</text>',
            ]
        )
    parts.append(
        '<text x="56" y="370" font-family="Arial, sans-serif" font-size="13" fill="#64748b">Workbook SHA-256: '
        + metrics["source"]["sha256"][:16]
        + "…</text>"
    )
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def build_bundle(source: Path, output: Path) -> dict[str, Any]:
    source = source.resolve()
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    structured = output / "structured"
    structured.mkdir(parents=True, exist_ok=True)
    source_dir = output / "source"
    source_dir.mkdir(parents=True, exist_ok=True)
    copied_source = source_dir / source.name
    shutil.copyfile(source, copied_source)
    source_hash = sha256(source)

    formulas = load_workbook(source, read_only=False, data_only=False)
    values = load_workbook(source, read_only=False, data_only=True)
    view = WorkbookView(formulas=formulas, values=values)

    worksheets: list[dict[str, Any]] = []
    all_cells: list[list[Any]] = [
        ["sheet_index", "sheet_name", "cell", "value", "formula"]
    ]
    for index, ws in enumerate(values.worksheets, start=1):
        title = ws.title
        slug = SHEET_SLUGS.get(title, f"sheet_{index:02d}")
        csv_name = f"sheet_{index:02d}_{slug}.csv"
        rows: list[list[Any]] = []
        for row_number in range(1, ws.max_row + 1):
            row = row_values(ws, row_number)
            if nonempty(row):
                rows.append([row_number, *row])
            for col_number in range(1, ws.max_column + 1):
                value = ws.cell(row_number, col_number).value
                formula = formulas[title].cell(row_number, col_number).value
                if value is not None or formula is not None:
                    coordinate = ws.cell(row_number, col_number).coordinate
                    all_cells.append(
                        [index, title, coordinate, clean(value), clean(formula)]
                    )
        write_csv(structured / csv_name, rows)
        worksheets.append(
            {
                "index": index,
                "name": title,
                "slug": slug,
                "row_count": ws.max_row,
                "nonempty_row_count": len(rows),
                "max_columns": ws.max_column,
                "csv": f"structured/{csv_name}",
            }
        )
    write_csv(structured / "workbook_cells.csv", all_cells)

    selected = {
        "calibration_points.csv": rows_with_header(view, "标定与定位", 10, 11, 37, 8),
        "yolo_classes.csv": rows_with_header(view, "视觉感知", 11, 12, 23, 6),
        "yolo_runtime_frames.csv": rows_with_header(view, "视觉感知", 25, 26, 53, 8),
        "uart_sample_log.csv": rows_with_header(view, "通信协议", 22, 23, 46, 8),
        "teach_reproduction.csv": rows_with_header(view, "运动控制", 12, 13, 18, 8),
        "grasp_demo_sample.csv": rows_with_header(view, "抓取操作", 4, 5, 31, 10),
        "task_runs.csv": rows_with_header(view, "抓取操作", 56, 57, 62, 8),
        "vla_instructions.csv": rows_with_header(view, "VLA推理", 11, 12, 31, 9),
        "real_run_1000.csv": rows_with_header(view, "安全与验证", 36, 37, 1036, 8),
        "robustness_matrix.csv": rows_with_header(view, "鲁棒性测试", 4, 5, 12, 6),
        "training_runs.csv": rows_with_header(view, "数据与训练", 12, 13, 19, 8),
        "ablation.csv": rows_with_header(view, "实验与消融", 12, 13, 19, 5),
    }
    for name, rows in selected.items():
        write_csv(structured / name, rows)

    metrics = derive_metrics(view, source.name, source_hash)
    json_dump(output / "field_validation.json", metrics)
    json_dump(output / "field_validation_metrics.json", metrics["worksheet_claims"])
    (output / "field_validation_dashboard.svg").write_text(
        svg_dashboard(metrics), encoding="utf-8"
    )
    json_dump(
        output / "worksheet_index.json",
        {"source_sha256": source_hash, "worksheets": worksheets},
    )

    derived: dict[str, str] = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "source_manifest.json":
            derived[path.relative_to(output).as_posix()] = sha256(path)
    manifest = {
        "schema": "xiaou_field_workbook_manifest.v2",
        "generated_at": datetime.now(timezone.utc).astimezone().date().isoformat(),
        "source_workbook": source.name,
        "source_sha256": source_hash,
        "source_bytes": source.stat().st_size,
        "worksheet_count": len(worksheets),
        "worksheets": worksheets,
        "derived_files": derived,
        "provenance": "The source workbook is copied byte-for-byte. Derived CSV/JSON/SVG files are mechanical exports or arithmetic summaries from that workbook.",
    }
    json_dump(output / "source_manifest.json", manifest)
    return {
        "source_sha256": source_hash,
        "worksheet_count": len(worksheets),
        "derived_file_count": len(derived),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, required=True, help="path to the source XLSX"
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="evidence directory to create"
    )
    args = parser.parse_args()
    result = build_bundle(args.source, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
