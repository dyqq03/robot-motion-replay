"""Motion package integration for local collision review; no ROS or network."""
from __future__ import annotations

from datetime import datetime
import json
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from collision_checker import CollisionChecker, DEFAULT_RULES, digest, require_pass

ROOT = Path(__file__).resolve().parents[1]


def require_new_package(directory):
    """Prevent a failed new trial from leaving an older ZIP under the same name."""
    directory = Path(directory)
    if directory.exists() or directory.with_name(directory.name + ".zip").exists():
        raise FileExistsError("Motion output already exists; use a new versioned name: " + str(directory))


def model_trajectory(model, baseline, poses):
    """Convert the exact preview model poses to a complete qpos path."""
    values = np.tile(model.qpos0, (len(poses), 1))
    addresses = {model.joint(j).name: int(model.jnt_qposadr[j]) for j in range(model.njnt)}
    for name, value in baseline.items():
        values[:, addresses[name]] = value
    for i, pose in enumerate(poses):
        for name, value in pose.items():
            values[i, addresses[name]] = value
    return values


def check_arm_trajectory(times, degrees, names, mapping, *, rules_path=DEFAULT_RULES):
    """Preflight for the shared arm exporter, before writing package files."""
    from calibrated_hand_pose import calibrated_hand_zero_pose
    from view_shadow_model import arm_positions
    model_path = ROOT / "generated/humanoid_shadow.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    poses = [arm_positions(dict(zip(names, np.deg2rad(row))), mapping) for row in degrees]
    path = model_trajectory(model, calibrated_hand_zero_pose(), poses)
    result = CollisionChecker(model, rules_path).check_trajectory(times, path)
    result["model_sha256"] = digest(model_path)
    result["uncommanded_pose_assumption"] = "calibrated hand zero; uncommanded body/neck at model zero"
    return result


def save_draft_report(report, name):
    """Keep failed trials useful without changing a published package."""
    directory = ROOT / "reports/collision_checks"
    directory.mkdir(parents=True, exist_ok=True)
    safe = Path(name).name
    path = directory / (safe + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return path


def check_package(directory, *, rules_path=None, stage="trajectory", at_seconds=None, checker=None):
    from preview_motion_package import load_package
    directory = Path(directory).resolve()
    rules_path = resolve_rules(directory, rules_path)
    before = source_hashes(directory)
    proxy_before = proxy_hashes(rules_path)
    model_hash = digest(ROOT / "generated/humanoid_shadow.xml")
    code_hashes = implementation_hashes()
    model, data, baseline, times, poses, meta = load_package(directory, check_collision_provenance=False, model=None if checker is None else checker.model)
    checker = checker or CollisionChecker(model, rules_path)
    values = model_trajectory(model, baseline, poses)
    if stage == "trajectory":
        report = checker.check_trajectory(times, values)
    elif stage in ("quick", "candidate"):
        seconds = float(times[0] if at_seconds is None else at_seconds)
        if not np.isfinite(seconds) or not times[0] <= seconds <= times[-1]:
            raise ValueError("Collision candidate time outside the motion")
        qpos = np.array([np.interp(seconds, times, values[:, j]) for j in range(model.nq)])
        checker.set_pose(qpos)
        report = (checker.quick(checker.data) if stage == "quick" else checker.candidate(checker.data))
        report.update(time_s=seconds, rules_sha256=checker.rules_sha256)
    else:
        raise ValueError("Invalid collision check stage")
    if (before != source_hashes(directory) or model_hash != digest(ROOT / "generated/humanoid_shadow.xml")
            or code_hashes != implementation_hashes() or checker.rules_sha256 != digest(rules_path)
            or proxy_before != proxy_hashes(rules_path)):
        raise ValueError("Collision inputs changed during review; retry with a stable model and package")
    report.update(package=directory.name, model_sha256=model_hash,
        source_hashes=before, implementation_sha256=code_hashes,
        uncommanded_pose_assumption="same baseline as local preview; uncommanded hands use calibrated zero",
        rules_file=str(Path(rules_path).resolve()),
        runtime_start_and_end_handoffs_checked=False, proxy_hashes=proxy_hashes(rules_path))
    return report


def source_hashes(directory):
    paths = {name: Path(directory) / name for name in ("fullbody.hdf5", "fullbody_meta.json", "motion.json")}
    for name in ("arm_mapping.json", "body_mapping.json", "neck_mapping.json", "hand_mapping.json",
                 "hand_hardware.json", "hand_calibration.json", "motion_baseline.json"):
        paths["configs/" + name] = ROOT / "configs" / name
    for side in ("left", "right"):
        for name in ("config.yaml", "calibration.yaml"):
            key = f"configs/hand_models/orcahand_v1_{side}/{name}"
            paths[key] = ROOT / key
    for name in ("generated/model_report.json", "URDF_90kg_with_hands/urdf/humanoid_with_hands.urdf"):
        paths[name] = ROOT / name
    # An unchanged XML does not imply unchanged mesh bytes. Bind the actual
    # external assets loaded by MuJoCo, including simplified wrist meshes.
    model_path = ROOT / "generated/humanoid_shadow.xml"
    robot = ET.parse(model_path).getroot()
    if robot.find(".//include") is not None:
        raise ValueError("Collision provenance requires a flattened MJCF without includes")
    compiler = robot.find("compiler")
    options = {} if compiler is None else compiler.attrib
    for node in robot.iter():
        if "file" not in node.attrib:
            continue
        if node.tag not in ("mesh", "texture", "hfield"):
            raise ValueError("Unsupported external model asset: " + node.tag)
        folder = options.get("texturedir" if node.tag == "texture" else "meshdir", options.get("assetdir", ""))
        asset = (model_path.parent / folder / node.attrib["file"]).resolve()
        paths["model_asset/" + str(asset.relative_to(ROOT))] = asset
    return {name: digest(path) for name, path in paths.items()}


def resolve_rules(directory, rules_path=None):
    if rules_path is not None:return Path(rules_path)
    manifest=json.loads((Path(directory)/'manifest.json').read_text())
    filename=manifest.get('collision_rules_file')
    if filename:
        if Path(filename).name!=filename:raise ValueError('Invalid package rules path')
        return Path(directory)/filename
    return DEFAULT_RULES


def proxy_hashes(rules_path):
    from collision_proxy import geometry_hashes
    from collision_checker import load_rules
    return geometry_hashes(load_rules(rules_path))


def implementation_hashes():
    result = {name: digest(ROOT / "scripts" / name) for name in (
        "collision_checker.py", "collision_proxy.py", "motion_collision.py", "preview_motion_package.py",
        "view_shadow_model.py", "calibrated_hand_pose.py", "joint_calibration.py", "model_paths.py")}
    result["unified_ui/app/hand_io.py"] = digest(ROOT / "unified_ui/app/hand_io.py")
    return result


def revalidation_path(directory):
    """Keep a fresh complete recheck outside immutable historical packages."""
    revision = hashlib.sha256(json.dumps(implementation_hashes(), sort_keys=True).encode()).hexdigest()
    return ROOT / "reports/collision_revalidations" / revision / Path(directory).name / "collision_validation.json"


def _report_matches(report, directory, manifest, rules_path):
    return (report.get("model_sha256") == manifest["model_sha256"] and
            report.get("source_hashes") == source_hashes(directory) and
            report.get("rules_sha256") == digest(rules_path) and
            report.get("implementation_sha256") == implementation_hashes() and
            report.get("mujoco_version") == mujoco.__version__ and
            report.get("proxy_hashes", {}) == proxy_hashes(rules_path) and
            report.get("certified_fraction") == 1.0)


def validate_package_report(directory, manifest, *, rules_path=None):
    """Cheap provenance validation; ordinary previews do not rerun mesh scans."""
    filename = "collision_validation.json"
    if filename not in manifest["files"]:
        raise ValueError("Required collision report missing from package manifest")
    rules_path=resolve_rules(directory,rules_path)
    report = json.loads((Path(directory) / filename).read_text())
    require_pass(report)
    if _report_matches(report, directory, manifest, rules_path):
        return report
    # A loader update invalidates old proofs. Accept only an ACTUAL full new
    # trajectory pass, with the same checks of every input/dependency hash.
    # Never relabel the embedded historical report or touch its ZIP.
    external = revalidation_path(directory)
    if external.is_file():
        refreshed = json.loads(external.read_text())
        require_pass(refreshed)
        if _report_matches(refreshed, directory, manifest, rules_path):
            return refreshed
    raise ValueError("Collision report is stale: recheck the current model, trajectory and rules")


def finalize_package(directory, *, rules_path=None, checker=None):
    """Recheck the FINAL arms/hands/neck streams and bind a passed report.

    Call after every HDF5/metadata edit and before creating the final ZIP. Failed
    drafts are retained for adjustment. Existing packages are never upgraded by
    the read-only CLI; only explicitly called generation code uses this method.
    """
    directory = Path(directory)
    report = check_package(directory, rules_path=rules_path, checker=checker)
    # Always preserve diagnostics, including failed or incomplete reviews.
    path = directory / "collision_validation.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    manifest_path = directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["draft"] = report["status"] != "passed"
    manifest["collision_check_required"] = report["status"] == "passed"
    manifest["files"][path.name] = digest(path)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    require_pass(report)
    return report
