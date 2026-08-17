from __future__ import annotations

import argparse
import math
import sys


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--min_r", type=float, default=0.05)
    parser.add_argument("--max_r", type=float, default=0.60)
    parser.add_argument("--zmin", type=float, default=0.0)
    parser.add_argument("--zmax", type=float, default=0.60)
    args = parser.parse_args()
    r = math.hypot(args.x, args.y)
    ok = args.min_r <= r <= args.max_r and args.zmin <= args.z <= args.zmax
    print("reachability:", ok, "r=", r)
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
