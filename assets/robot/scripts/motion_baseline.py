"""User-specified hardware start/end pose; independent of calibration zero."""
import json
import hashlib
from pathlib import Path

import numpy as np

PATH = Path(__file__).resolve().parents[1] / "configs/motion_baseline.json"
ARM_PATH = PATH.with_name("arm_motion_defaults.json")
NECK_PATH = PATH.with_name("neck_motion_defaults.json")
HAND_PATH = PATH.with_name("hand_motion_defaults.json")


def read_baseline():
    # Preserve the immutable historical baseline referenced by old packages.
    # Current generation follows the separately versioned user-approved pose.
    config = json.loads((ARM_PATH if ARM_PATH.exists() else PATH).read_text())
    if config["unit"] != "deg" or config["arm_joint_order"] != [1, 2, 3, 4, 5, 7]:
        raise ValueError("Unexpected baseline units or joint ordering")
    return config


def arm_baseline_deg():
    config = read_baseline()
    return {f"openarm_{side}_joint{j}": float(value)
            for side in ("left", "right")
            for j, value in zip(config["arm_joint_order"], config["arms"][side], strict=True)}


def neck_baseline_deg():
    # Generation defaults evolve independently from immutable historical
    # calibration/baseline records referenced by published collision reports.
    config = json.loads(NECK_PATH.read_text()) if NECK_PATH.exists() else read_baseline()
    if config["unit"] != "deg" or set(config["neck"]) != {"neck_joint1", "neck_joint2"}:
        raise ValueError("Unexpected neck baseline units or names")
    return {name: float(value) for name, value in config["neck"].items()}


def hand_baseline_deg(side=None):
    """Named absolute hand joint degrees, independent of calibrated zero.

    With a side return 16 unprefixed names; otherwise 32 side/name keys. A
    changed calibration must not silently reinterpret a measured rest pose.
    """
    config = json.loads(HAND_PATH.read_text())
    if config["unit"] != "deg" or config["coordinate_system"] != "calibrated_hardware_joint_absolute_angles":
        raise ValueError("Unexpected hand motion baseline coordinates")
    names = {"thumb_" + j for j in ("mcp", "abd", "pip", "dip")} | {
        finger + "_" + joint for finger in ("index", "middle", "ring", "pinky")
        for joint in ("abd", "mcp", "pip")}
    if set(config["hands"]) != {"left", "right"}:
        raise ValueError("Hand baseline must include both hands")
    result = {}
    for hand in ("left", "right"):
        pose = config["hands"][hand]
        if set(pose) != names or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) for v in pose.values()):
            raise ValueError("Expected 16 finite named hand baseline angles: " + hand)
        for filename, expected in config["calibration_files_sha256"][hand].items():
            path = PATH.parent / "hand_models" / ("orcahand_v1_" + hand) / filename
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError("Hand baseline calibration changed; acquire a new measured reference: " + str(path))
        result[hand] = {name: float(value) for name, value in pose.items()}
    if side is not None:
        if side not in result:
            raise ValueError("Unknown hand side: " + str(side))
        return result[side]
    return {hand + "/" + name: value for hand, pose in result.items() for name, value in pose.items()}


def require_hand_endpoints(degrees, names, *, side):
    """Verify a commanded hand starts/ends at the user's measured pose."""
    pose = hand_baseline_deg(side)
    if len(names) != 16 or set(names) != set(pose):
        raise ValueError("Expected all 16 named hand joints")
    q = np.asarray(degrees)
    if q.ndim != 2 or len(q) < 2 or q.shape[1] != 16 or not np.isfinite(q).all():
        raise ValueError("Invalid hand baseline trajectory dimensions or values")
    expected = np.array([pose[name] for name in names])
    if not np.allclose(q[[0, -1]], expected, rtol=0, atol=1e-9):
        raise ValueError(f"{side} hand endpoints must match {HAND_PATH}")


def require_endpoints(degrees, names, *, group="arms"):
    pose = arm_baseline_deg() if group == "arms" else neck_baseline_deg() if group == "neck" else None
    if pose is None:
        raise ValueError(f"Unknown baseline group: {group}")
    expected = np.array([pose[name] for name in names])
    q = np.asarray(degrees)
    if q.ndim != 2 or len(q) < 2 or q.shape[1] != len(names) or not np.isfinite(q).all():
        raise ValueError("Invalid baseline trajectory dimensions or values")
    if not np.allclose(q[[0, -1]], expected, rtol=0, atol=1e-9):
        raise ValueError(f"{group} endpoints must match {PATH}")
