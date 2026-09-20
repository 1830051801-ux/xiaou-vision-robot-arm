#!/usr/bin/env python3
"""Collect existing XiaoU experiment evidence and build a portable Release bundle.

This is an archival operation. It never runs or relabels an experiment. Public
JSON copies normalize local paths and keep hashes linking them to source bytes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def portable(value: object, mappings: list[tuple[str, str]]) -> object:
    if isinstance(value, dict):
        return {key: portable(item, mappings) for key, item in value.items()}
    if isinstance(value, list):
        return [portable(item, mappings) for item in value]
    if isinstance(value, str):
        result = value.replace("\\", "/")
        for source, target in mappings:
            result = result.replace(source.replace("\\", "/"), target)
        if re.match(r"^[A-Za-z]:/", result):
            return "local-artifact/" + result.rsplit("/", 1)[-1]
        if result.startswith("/home/"):
            return "device-artifact/" + result.rsplit("/", 1)[-1]
        result = re.sub(r"(?<![A-Za-z])[A-Za-z]:/+[^\s\"'<>]+", "local-artifact/redacted-path", result)
        return result
    return value


def json_bytes(value: object) -> bytes:
    return (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-source", type=Path, required=True)
    parser.add_argument("--research-source", type=Path, required=True)
    parser.add_argument("--evidence-output", type=Path, required=True)
    parser.add_argument("--bundle-output", type=Path, required=True)
    args = parser.parse_args()
    pi = args.pi_source.resolve()
    research = args.research_source.resolve()
    output = args.evidence_output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    args.bundle_output.parent.mkdir(parents=True, exist_ok=True)
    mappings = [(str(pi), "raspberry_pi"), (str(research), "research/embodied-arm-learning")]
    groups = [
        (pi / "runtime/simulations", "pi_simulations"),
        (pi / "runtime/decision", "pi_decision"),
        (pi / "tmp/verification_final_20260816", "pi_final_verification"),
        (research / "runtime/constraint_diffusion", "digital_twin"),
    ]
    records: list[dict[str, object]] = []
    excluded: list[dict[str, str]] = []
    for source, label in groups:
        for path in sorted(source.rglob("*.json")):
            if "python_site" in path.parts:
                continue
            raw = path.read_bytes()
            try:
                value = json.loads(raw)
            except (UnicodeError, json.JSONDecodeError):
                excluded.append({"path": (Path(label) / path.relative_to(source)).as_posix(), "reason": "file is not parseable JSON; often a redirected console log"})
                continue
            relative = Path(label) / path.relative_to(source)
            public = json_bytes(portable(value, mappings))
            destination = output / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(public)
            records.append({
                "path": relative.as_posix(), "source_sha256": digest(raw),
                "public_sha256": digest(public), "bytes": len(public),
            })
    asset_groups = [
        (pi / "runtime/decision", "raspberry_pi/runtime/decision", {".pt", ".onnx", ".json"}),
        (pi / "models", "raspberry_pi/models", {".onnx", ".names"}),
        (research / "runtime/constraint_diffusion", "research/embodied-arm-learning/runtime/constraint_diffusion", {".pt", ".npz", ".json", ".png"}),
    ]
    assets: list[dict[str, object]] = []
    seen: dict[str, str] = {}
    with zipfile.ZipFile(args.bundle_output, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for source, label, suffixes in asset_groups:
            for path in sorted(source.glob("*")):
                if not path.is_file() or path.suffix.lower() not in suffixes:
                    continue
                data = path.read_bytes()
                if path.suffix == ".json":
                    try:
                        data = json_bytes(portable(json.loads(data), mappings))
                    except (UnicodeError, json.JSONDecodeError):
                        excluded.append({"path": f"{label}/{path.name}", "reason": "file is not parseable JSON; original retained locally"})
                        continue
                name = f"{label}/{path.name}"
                checksum = digest(data)
                if checksum not in seen:
                    archive.writestr(name, data)
                    seen[checksum] = name
                assets.append({"path": name, "sha256": checksum, "bytes": len(data), "stored_as": seen[checksum]})
        manifest = {
            "schema": "xiaou_offline_archive_v1", "archive_date": "2026-09-20",
            "evidence_origin": "historical offline experiments; dates remain in original report names",
            "new_training_performed": False, "hardware_connected": False,
            "json_path_normalization": "local absolute paths replaced by repository-relative or artifact names",
            "reports": records, "assets": assets, "excluded": excluded,
        }
        archive.writestr("MANIFEST.json", json_bytes(manifest))
        archive.writestr("README.txt", (
            "XiaoU offline experiment assets\n\n"
            "Historical simulation, training and quantization artifacts. Not hardware measurements.\n"
            "Duplicate content is stored once; MANIFEST.json maps every original name to stored_as.\n"
            "Extract under the repository root. Copy stored_as to path only when an alias is needed.\n"
            "Research datasets are synthetic. The public robot configuration stays motion locked.\n"
        ))
    (output / "MANIFEST.json").write_bytes(json_bytes(manifest))
    print(json.dumps({"reports": len(records), "asset_names": len(assets), "unique_assets": len(seen), "bundle_bytes": args.bundle_output.stat().st_size}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
