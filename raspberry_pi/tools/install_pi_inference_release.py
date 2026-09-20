#!/usr/bin/env python3
"""Stage, verify, activate, or roll back a no-motion Pi inference release.

The program only manipulates release directories and symlinks.  It never opens
serial/CAN/ROS hardware, starts a controller, or sends an actuator command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile
from typing import Any

try:
    from .verify_pi_inference_release import verify
except ImportError:  # Direct invocation from tools/ on a Pi.
    from verify_pi_inference_release import verify


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_member(member: tarfile.TarInfo) -> Path:
    pure = PurePosixPath(member.name)
    if not member.name or pure.is_absolute() or ".." in pure.parts or member.issym() or member.islnk() or member.isdev():
        raise ValueError(f"unsafe archive member: {member.name}")
    return Path(*pure.parts)


def _replace_link(link: Path, target: Path) -> None:
    if (link.exists() or link.is_symlink()) and not link.is_symlink():
        raise RuntimeError(f"refusing to replace non-symlink release pointer: {link}")
    temporary = link.with_name(link.name + ".next")
    if temporary.exists() or temporary.is_symlink():
        temporary.unlink()
    os.symlink(target, temporary, target_is_directory=True)
    os.replace(temporary, link)


def _rollback(release_root: Path) -> dict[str, Any]:
    current = release_root / "current"
    previous = release_root / "previous"
    if not previous.is_symlink():
        raise RuntimeError("no previous release is available for rollback")
    if (current.exists() or current.is_symlink()) and not current.is_symlink():
        raise RuntimeError("current release pointer is not a symlink")
    target = previous.resolve()
    if not target.is_dir():
        raise RuntimeError("previous release target is missing")
    if current.is_symlink():
        _replace_link(previous, current.resolve())
    _replace_link(current, target)
    return {"action": "rollback", "active_release": str(target), "hardware_motion": False}


def install(archive: Path, release_root: Path, *, expected_sha256: str, activate: bool) -> dict[str, Any]:
    archive = archive.resolve()
    release_root = release_root.expanduser().resolve()
    if not archive.is_file():
        raise FileNotFoundError(archive)
    actual_sha256 = _sha256(archive)
    if actual_sha256.lower() != expected_sha256.strip().lower():
        raise RuntimeError("archive SHA-256 does not match the explicitly supplied value")
    release_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".xiaou_stage_", dir=release_root) as temporary:
        stage = Path(temporary)
        with tarfile.open(archive, "r:*") as bundle:
            members = bundle.getmembers()
            for member in members:
                relative = _safe_member(member)
                if not relative.parts or relative.parts[0] != "xiaou_pi":
                    raise ValueError("archive contains content outside xiaou_pi")
                destination = (stage / relative).resolve()
                destination.relative_to(stage)
                try:
                    bundle.extract(member, stage, filter="data")
                except TypeError:  # Python < 3.12; member has already passed strict checks.
                    bundle.extract(member, stage)
        candidate = stage / "xiaou_pi"
        if not candidate.is_dir() or any(path.name != "xiaou_pi" for path in [candidate]):
            raise RuntimeError("archive must contain exactly the xiaou_pi release root")
        verification = verify(candidate, require_runtime_modules=False)
        if not verification["passed"]:
            raise RuntimeError("staged release verification failed: " + ",".join(verification["failures"]))
        release_name = "release-" + actual_sha256[:16]
        destination = release_root / release_name
        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() or not destination.is_dir():
                raise RuntimeError(f"release path already exists and is not a directory: {destination}")
            existing_verification = verify(destination, require_runtime_modules=False)
            if not existing_verification["passed"]:
                raise RuntimeError("existing release verification failed: " + ",".join(existing_verification["failures"]))
            verification = existing_verification
        else:
            shutil.move(str(candidate), str(destination))
    result: dict[str, Any] = {
        "action": "activated" if activate else "staged",
        "release": str(destination),
        "archive": str(archive),
        "archive_sha256": actual_sha256,
        "verification": verification,
        "hardware_motion": False,
    }
    if activate:
        current = release_root / "current"
        previous = release_root / "previous"
        if current.is_symlink():
            _replace_link(previous, current.resolve())
        _replace_link(current, destination)
        result["active_release"] = str(destination)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--sha256", help="required archive digest; no implicit trust")
    parser.add_argument("--release-root", type=Path, default=Path.home() / "xiaou_releases")
    parser.add_argument("--activate", action="store_true", help="atomically point current at the verified staged release")
    parser.add_argument("--rollback", action="store_true", help="atomically restore the previous release")
    args = parser.parse_args()
    if args.rollback:
        if args.archive or args.sha256 or args.activate:
            parser.error("--rollback cannot be combined with archive or activation arguments")
        result = _rollback(args.release_root.expanduser().resolve())
    else:
        if not args.archive or not args.sha256:
            parser.error("--archive and --sha256 are required for staging")
        result = install(args.archive, args.release_root, expected_sha256=args.sha256, activate=args.activate)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
