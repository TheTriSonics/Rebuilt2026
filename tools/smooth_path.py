"""Rewrite a PathPlanner path's control points as a minimum-acceleration spline.

Place only the waypoints in the PathPlanner GUI, then run:

    uv run python tools/smooth_path.py BumpyShoot
    uv run python tools/smooth_path.py BumpyShoot --lock-ends   # keep end handle directions
    uv run python tools/smooth_path.py --all

Only the control points change; waypoints, rotation targets, markers and
constraint zones are left alone. Reopen the path in PathPlanner afterwards.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utilities.path_smoothing import smooth_controls  # noqa: E402

PATHS_DIR = Path(__file__).resolve().parent.parent / "deploy" / "pathplanner" / "paths"


def _xy(point: dict) -> np.ndarray:
    return np.array([point["x"], point["y"]])


def smooth_file(path_file: Path, lock_ends: bool) -> None:
    data = json.loads(path_file.read_text())
    waypoints = data["waypoints"]
    if len(waypoints) < 2:
        print(f"{path_file.stem}: fewer than two waypoints, skipped")
        return
    anchors = np.array([_xy(w["anchor"]) for w in waypoints])

    start_dir = end_dir = None
    if lock_ends:
        start_dir = _xy(waypoints[0]["nextControl"]) - anchors[0]
        end_dir = anchors[-1] - _xy(waypoints[-1]["prevControl"])

    first, second = smooth_controls(anchors, start_dir, end_dir)
    for i in range(len(waypoints) - 1):
        waypoints[i]["nextControl"] = {"x": float(first[i][0]), "y": float(first[i][1])}
        waypoints[i + 1]["prevControl"] = {"x": float(second[i][0]), "y": float(second[i][1])}
    # Read by the auto-smooth PathPlanner fork; ignored by everything else
    data["autoSmooth"] = True
    data["autoSmoothLockEnds"] = lock_ends

    path_file.write_text(json.dumps(data, indent=2) + "\n")
    print(f"{path_file.stem}: smoothed {len(waypoints)} waypoints")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("names", nargs="*", help="path names (without .path)")
    parser.add_argument("--all", action="store_true", help="smooth every path")
    parser.add_argument(
        "--lock-ends",
        action="store_true",
        help="keep the current direction of the first and last control handles",
    )
    args = parser.parse_args()

    files = sorted(PATHS_DIR.glob("*.path")) if args.all else []
    files += [PATHS_DIR / f"{name}.path" for name in args.names]
    if not files:
        parser.error("give path names or --all")
    for f in files:
        smooth_file(f, args.lock_ends)


if __name__ == "__main__":
    main()
