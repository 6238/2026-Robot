"""
Generates "Upper" versions of Lower Trench paths by flipping across the
horizontal midline of the FRC 2026 Reefscape field (y = 8.052 / 2 = 4.026m).

Transformation:
  y_new   = FIELD_HEIGHT - y_old
  angle_new = -angle_old   (reflection across x-axis)
"""

import json
import copy
import os

FIELD_HEIGHT = 8.052
PATHS_DIR = os.path.join(os.path.dirname(__file__),
                         "src", "main", "deploy", "pathplanner", "paths")

# Maps old linkedName values to new ones
LINKED_NAME_MAP = {
    "Lower Trench Start": "Upper Trench Start",
    "Lower Trench Shoot": "Upper Trench Shoot",
    "Trench Delay Depot Start": "Upper Trench Delay Depot Start",
}

# Maps old folder names to new ones
FOLDER_MAP = {
    "Lower Trench 2.5": "Upper Trench 2.5",
    "Weird Trench":     "Upper Weird Trench",
}

# (input filename, output filename)
PATH_PAIRS = [
    ("Lower Trench Cycle 1.path",          "Upper Trench Cycle 1.path"),
    ("Lower Trench Cycle 2.path",          "Upper Trench Cycle 2.path"),
    ("Lower Trench Cycle 3.path",          "Upper Trench Cycle 3.path"),
    ("Lower Trench Delay Path.path",       "Upper Trench Delay Path.path"),
    ("Lower Risky Trench Delay Path.path", "Upper Risky Trench Delay Path.path"),
]


def flip_y(val):
    return FIELD_HEIGHT - val


def flip_angle(deg):
    return -deg


def flip_point(pt):
    if pt is None:
        return None
    return {"x": pt["x"], "y": flip_y(pt["y"])}


def flip_path(data):
    out = copy.deepcopy(data)

    # Flip waypoints
    for wp in out["waypoints"]:
        wp["anchor"]      = flip_point(wp["anchor"])
        wp["prevControl"] = flip_point(wp["prevControl"])
        wp["nextControl"] = flip_point(wp["nextControl"])
        if wp.get("linkedName") in LINKED_NAME_MAP:
            wp["linkedName"] = LINKED_NAME_MAP[wp["linkedName"]]

    # Flip rotation targets
    for rt in out.get("rotationTargets", []):
        rt["rotationDegrees"] = flip_angle(rt["rotationDegrees"])

    # Flip goal end state rotation
    if "goalEndState" in out:
        out["goalEndState"]["rotation"] = flip_angle(out["goalEndState"]["rotation"])

    # Flip ideal starting state rotation
    if "idealStartingState" in out:
        out["idealStartingState"]["rotation"] = flip_angle(out["idealStartingState"]["rotation"])

    # Remap folder
    if out.get("folder") in FOLDER_MAP:
        out["folder"] = FOLDER_MAP[out["folder"]]

    return out


def main():
    for src_name, dst_name in PATH_PAIRS:
        src_path = os.path.join(PATHS_DIR, src_name)
        dst_path = os.path.join(PATHS_DIR, dst_name)

        with open(src_path, "r") as f:
            data = json.load(f)

        flipped = flip_path(data)

        with open(dst_path, "w") as f:
            json.dump(flipped, f, indent=2)

        print(f"  {src_name}  ->  {dst_name}")

    print("\nDone.")


if __name__ == "__main__":
    main()
