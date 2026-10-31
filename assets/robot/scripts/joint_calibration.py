"""Offline calibration helpers for the separate body, neck and hand configs."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET
from model_paths import SOURCE_URDF

ROOT = Path(__file__).resolve().parents[1]
SOURCE = SOURCE_URDF
HAND_SUFFIXES = (
    "ThumbMCP", "ThumbAbduction", "ThumbPIP", "ThumbDIP",
    "IndexAbduction", "IndexMCP", "IndexPIP", "MiddleAbduction", "MiddleMCP", "MiddlePIP",
    "RingAbduction", "RingMCP", "RingPIP", "PinkyAbduction", "PinkyMCP", "PinkyPIP",
)
GROUP_MODELS = {
    "body": {"WaistYaw", "HipPitch", "Knee", "AnklePitch"},
    "neck": {"NeckYaw", "NeckPitch"},
    "hand": {side + suffix for side in ("Left", "Right") for suffix in HAND_SUFFIXES},
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_hand_sources(mapping,root=ROOT):
    """A model must not silently use a different encoder calibration bundle."""
    revision=mapping.get('hardware_calibration_revision')
    if not revision:return
    root=Path(root)
    for name,key in [('hand_hardware.json','calibration_revision'),('hand_calibration.json','revision')]:
        if json.loads((root/'configs'/name).read_text()).get(key)!=revision:
            raise ValueError('手部校准版本不一致；按技能 references/hand-reference.md 导入一致的标定文件，再重新采集参考手型')
    for side,source in mapping['hardware_calibration_sources'].items():
        for name,expected in source['parameter_sha256'].items():
            path=root/source['model_path']/name
            if sha256(path)!=expected:raise ValueError('手部校准源文件已变化：'+side+'/'+name+'；请统一更新配置和模型报告')


def to_radians(value, unit):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Joint values must be finite")
    if unit == "deg":
        return math.radians(value)
    if unit == "rad":
        return value
    raise ValueError(f"Unsupported unit: {unit}")


def validate_specs(mapping, group):
    if mapping.get("group") != group or group not in GROUP_MODELS:
        raise ValueError(f"Wrong mapping group: expected {group}")
    specs = mapping["joints"]
    source_names = [s["source_joint"] for s in specs]
    model_names = [s["model_joint"] for s in specs]
    if (len(set(source_names)) != len(source_names) or len(set(model_names)) != len(model_names)
            or set(model_names) != GROUP_MODELS[group]):
        raise ValueError(f"Expected exactly the {len(GROUP_MODELS[group])} unique {group} joints")
    for s in specs:
        if s["sign"] not in (-1, 1) or not math.isfinite(s["offset_rad"]):
            raise ValueError(f"Invalid sign/offset: {s['source_joint']}")
        low, high = map(float, s["real_limits_rad"])
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError(f"Invalid real limits: {s['source_joint']}")
        to_radians(0, s["source_unit"])
    return specs


def mapped_positions(positions, mapping):
    """Map named native hardware values (body/hand deg, neck rad) to model rad."""
    specs = {s["source_joint"]: s for s in validate_specs(mapping, mapping["group"])}
    if set(positions) - set(specs):
        raise ValueError(f"Unknown {mapping['group']} hardware joint names")
    result = {}
    for name, value in positions.items():
        s = specs[name]
        raw = to_radians(value, s["source_unit"])
        low, high = s["real_limits_rad"]
        if not low - 1e-12 <= raw <= high + 1e-12:
            raise ValueError(f"Hardware pose outside limits: {name}={value}")
        result[s["model_joint"]] = s["sign"] * raw + s["offset_rad"]
    return result


def audit_mapping(robot, mapping, group, tolerance=1e-6):
    audit = []
    for s in validate_specs(mapping, group):
        limit = robot.find(f"joint[@name='{s['model_joint']}']/limit")
        if limit is None:
            raise ValueError(f"URDF joint/limit missing: {s['model_joint']}")
        actual = [float(limit.get(k)) for k in ("lower", "upper")]
        expected = sorted(s["sign"] * v + s["offset_rad"] for v in s["real_limits_rad"])
        stored = s["model_limits_rad"]
        if (len(stored) != 2 or any(abs(a - e) > tolerance for a, e in zip(actual, expected))
                or any(not math.isfinite(v) for v in stored)
                or any(abs(a - e) > tolerance for a, e in zip(stored, expected))):
            raise ValueError(f"{group} mapping/URDF limit mismatch: {s['model_joint']}; inspect bundled configs and URDF using references/environment.md")
        if "zero_pose_real_rad" in s and abs(s["sign"] * s["zero_pose_real_rad"] + s["offset_rad"]) > 1e-10:
            raise ValueError(f"Saved zero reading does not map to zero: {s['model_joint']}")
        audit.append({**s, "urdf_limits_rad": actual})
    return audit


def calibrate_group(group, source=SOURCE, mapping_path=None, readings_path=None):
    mapping_path = Path(mapping_path or ROOT / f"configs/{group}_mapping.json").resolve()
    readings_path = Path(readings_path or ROOT / f"configs/{group}_zero_readings.json").resolve()
    source = Path(source).resolve()
    if source.suffix != ".urdf":
        raise ValueError("Select the active .urdf, not a backup")
    mapping = json.loads(mapping_path.read_text())
    specs = validate_specs(mapping, group)
    readings = json.loads(readings_path.read_text())
    if readings.get("group") != group:
        raise ValueError(f"Wrong zero readings group: expected {group}")
    verified = readings.get("physical_zero_verified", False)
    if not isinstance(verified, bool):
        raise ValueError("physical_zero_verified must be a JSON boolean")
    if set(readings["positions"]) != {s["source_joint"] for s in specs}:
        raise ValueError(f"Zero readings must contain exactly all {group} source joint names")
    outside = []
    for s in specs:
        zero = to_radians(readings["positions"][s["source_joint"]], readings["unit"])
        s["offset_rad"] = -s["sign"] * zero
        # Use persisted raw limits; never re-read shifted URDF bounds as raw limits.
        s["model_limits_rad"] = sorted(s["sign"] * v + s["offset_rad"] for v in s["real_limits_rad"])
        s["zero_pose_real_rad"] = zero
        s["zero_reading_within_real_limits"] = s["real_limits_rad"][0] <= zero <= s["real_limits_rad"][1]
        if not s["zero_reading_within_real_limits"]:
            outside.append(s["source_joint"])
    mapping.update(offset_basis="offset=-sign*recorded_hardware_position_at_URDF_zero",
                   physical_zero_verified=verified, zero_readings_file=str(readings_path),
                   zero_readings_sha256=sha256(readings_path), zero_source=readings.get("source", "user readings"))
    by_name = {s["model_joint"]: s for s in specs}
    original = source.read_text()
    changed = set()

    def update_joint(match):
        block = match.group(0)
        name = ET.fromstring(block).get("name")
        if name not in by_name:
            return block

        def update_limit(limit_match):
            tag = limit_match.group(0)
            for key, value in zip(("lower", "upper"), by_name[name]["model_limits_rad"]):
                tag, count = re.subn(r'\b' + key + r'\s*=\s*([\'"]).*?\1',
                                     lambda _: f'{key}="{value:.12f}"', tag)
                if count != 1:
                    raise ValueError(f"Expected one {key} limit: {name}")
            return tag

        block, count = re.subn(r"<limit\b[^>]*>", update_limit, block)
        if count != 1:
            raise ValueError(f"Expected one limit tag: {name}")
        changed.add(name)
        return block

    updated = re.sub(r"<joint\b[^>]*>.*?</joint>", update_joint, original, flags=re.DOTALL)
    if changed != set(by_name):
        raise ValueError("Not all selected group joints were updated")
    robot = ET.fromstring(updated)
    audit = audit_mapping(robot, mapping, group)
    reports = ROOT / "reports"
    report_path = reports / f"{group}_limits_comparison.json"
    report_md = reports / f"{group}_limits_comparison.md"
    backup = ROOT / "backups" / (group + "_zero_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    backup.mkdir(parents=True)
    for p in (source, mapping_path, readings_path, report_path, report_md):
        if p.is_file():
            shutil.copy2(p, backup / p.name)
    source.write_text(updated)
    mapping_path.write_text(json.dumps(mapping, indent=2, ensure_ascii=False) + "\n")
    reports.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps({"group": group, "source_urdf": str(source),
        "source_sha256": sha256(source), "physical_zero_verified": verified,
        "zero_source": mapping["zero_source"], "backup": str(backup), "joints": audit},
        indent=2, ensure_ascii=False) + "\n")
    rows = [f"# {group} 零位与限位", "", "由独立校准脚本生成。真机基准限位首次复制自校准前 URDF，后续从 JSON 读取。", "",
            "| 真机关节 | 模型关节 | sign | 零姿态真机读数 rad | offset rad | 模型下限 rad | 模型上限 rad |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for s in specs:
        rows.append(f"| {s['source_joint']} | {s['model_joint']} | {s['sign']:+d} | {s['zero_pose_real_rad']:.12f} | "
                    f"{s['offset_rad']:.12f} | {s['model_limits_rad'][0]:.12f} | {s['model_limits_rad'][1]:.12f} |")
    rows += ["", f"备份：{backup}", "", f"超出真机基准限位的零读数：{outside}"]
    report_md.write_text("\n".join(rows) + "\n")
    print(json.dumps({"group": group, "mapping": str(mapping_path), "urdf": str(source),
        "backup": str(backup), "joint_count": len(specs), "zero_readings_outside_real_limits": outside,
        "next_step": "python scripts/build_mujoco_model.py"}, indent=2, ensure_ascii=False))
    return mapping


def main(group):
    parser = argparse.ArgumentParser(description=f"Calibrate the {group} group offline and synchronize URDF limits")
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--mapping", type=Path, default=ROOT / f"configs/{group}_mapping.json")
    parser.add_argument("--zero-readings", type=Path, default=ROOT / f"configs/{group}_zero_readings.json")
    args = parser.parse_args()
    calibrate_group(group, args.source, args.mapping, args.zero_readings)
