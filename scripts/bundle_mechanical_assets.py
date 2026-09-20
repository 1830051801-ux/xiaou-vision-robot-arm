#!/usr/bin/env python3
"""Archive the original STEP and deduplicated geometry without changing source files."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from collect_offline_artifacts import portable, json_bytes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest-output", type=Path, required=True)
    args = parser.parse_args()
    root = args.geometry.resolve()
    files = [args.step] + sorted(
        (path for path in root.rglob("*") if path.is_file()),
        key=lambda path: (0 if "world_model" in path.parts else 1, path.as_posix()),
    )
    seen: dict[str, str] = {}
    records = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            name = "source/robot-assembly.step" if path == args.step else "geometry/" + path.relative_to(root).as_posix()
            data = path.read_bytes()
            original_hash = hashlib.sha256(data).hexdigest()
            if path.suffix == ".json":
                data = json_bytes(portable(json.loads(data.decode("utf-8-sig")), []))
            checksum = hashlib.sha256(data).hexdigest()
            if checksum not in seen:
                archive.writestr(name, data)
                seen[checksum] = name
            records.append({"path": name, "bytes": len(data), "source_sha256": original_hash, "sha256": checksum, "stored_as": seen[checksum]})
        manifest = {
            "schema": "xiaou_mechanical_archive_v1", "archive_date": "2026-09-20",
            "mesh_units": "mm", "urdf_scale": 0.001,
            "scope": "original CAD and static home-pose visual exports; not measured dynamics or certified collision geometry",
            "files": records, "unique_files": len(seen),
        }
        archive.writestr("MANIFEST.json", json_bytes(manifest))
        archive.writestr("README.txt", "Original XiaoU STEP and geometry exports.\nDuplicate byte content is stored once. MANIFEST.json records aliases in stored_as.\nThe geometry/world_model directory is the primary static visual assembly.\nUnits: mm; URDF scale: 0.001. See MANIFEST.json for scope.\n")
    args.manifest_output.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_output.write_bytes(json_bytes(manifest))
    print(json.dumps({"source_files": len(records), "unique_files": len(seen), "bundle_bytes": args.output.stat().st_size}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
