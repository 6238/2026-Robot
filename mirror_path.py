"""
Mirror PathPlanner path files across Y = 4.065m (field center).

Usage:
  # Mirror a single file (writes to <name>_mirrored.path)
  python mirror_path.py "path/to/MyPath.path"

  # Mirror a single file to a specific output
  python mirror_path.py "path/to/MyPath.path" "path/to/Output.path"

  # Mirror all .path files in a directory
  python mirror_path.py path/to/paths/
"""

import json
import sys
from pathlib import Path

FIELD_CENTER_Y = 4.065


def mirror_y(y: float) -> float:
    return 2 * FIELD_CENTER_Y - y


def mirror_rotation(deg: float) -> float:
    return -deg


def mirror_point(pt: dict | None) -> dict | None:
    if pt is None:
        return None
    return {"x": pt["x"], "y": mirror_y(pt["y"])}


def mirror_path(data: dict) -> dict:
    # Waypoint bezier control points
    for wp in data.get("waypoints", []):
        wp["anchor"] = mirror_point(wp["anchor"])
        wp["prevControl"] = mirror_point(wp.get("prevControl"))
        wp["nextControl"] = mirror_point(wp.get("nextControl"))

    # Rotation targets
    for rt in data.get("rotationTargets", []):
        rt["rotationDegrees"] = mirror_rotation(rt["rotationDegrees"])

    # Goal end state rotation
    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] = mirror_rotation(
            data["goalEndState"]["rotation"]
        )

    # Ideal starting state rotation
    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] = mirror_rotation(
            data["idealStartingState"]["rotation"]
        )

    return data


def process_file(src: Path, dst: Path):
    data = json.loads(src.read_text(encoding="utf-8"))
    mirrored = mirror_path(data)
    dst.write_text(json.dumps(mirrored, indent=2), encoding="utf-8")
    print(f"  {src.name}  →  {dst.name}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    src_arg = Path(sys.argv[1])

    if src_arg.is_dir():
        files = list(src_arg.glob("*.path"))
        if not files:
            print(f"No .path files found in {src_arg}")
            sys.exit(1)
        print(f"Mirroring {len(files)} file(s) in {src_arg}:")
        for f in files:
            dst = f.with_stem(f.stem + "_mirrored")
            process_file(f, dst)
    else:
        if not src_arg.exists():
            print(f"File not found: {src_arg}")
            sys.exit(1)
        if len(sys.argv) >= 3:
            dst = Path(sys.argv[2])
        else:
            dst = src_arg.with_stem(src_arg.stem + "_mirrored")
        print("Mirroring:")
        process_file(src_arg, dst)


if __name__ == "__main__":
    main()
