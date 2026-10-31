"""Preview the generated shadow model without connecting to robot hardware."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import mujoco

from joint_calibration import mapped_positions
from calibrated_hand_pose import CalibratedHandPose

ROOT = Path(__file__).resolve().parents[1]


def arm_positions(real_positions, mapping):
    result = {}
    specs = {row["source_joint"]: row for row in mapping["joints"]}
    unknown = set(real_positions) - set(specs)
    if unknown:
        raise ValueError(f"Unknown hardware joint names: {sorted(unknown)}")
    for name, raw in real_positions.items():
        value = float(raw)
        spec = specs[name]
        lower, upper = spec["real_limits_rad"]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"Invalid real joint position: {name}={value}")
        result[spec["model_joint"]] = spec["sign"] * value + spec["offset_rad"]
    return result


def apply_pose(model, data, positions, *, enforce_limits=True):
    updates = []
    for name, raw in positions.items():
        value = float(raw)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0 or not math.isfinite(value):
            raise ValueError(f"Invalid model joint position: {name}={raw}")
        low, high = model.jnt_range[jid]
        # Exported ranges can differ by a few microradians due to serialization.
        if enforce_limits and model.jnt_limited[jid] and not low - 1e-5 <= value <= high + 1e-5:
            raise ValueError(f"Joint outside model limits: {name}={value}")
        updates.append((model.jnt_qposadr[jid], value))
    for address, value in updates:
        data.qpos[address] = value
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


def fullbody_positions(grouped_positions, mappings):
    """Named values: arm/neck rad, body/hand deg; hand names include left/ or right/."""
    unknown = set(grouped_positions) - set(mappings)
    if unknown:
        raise ValueError(f"Unknown hardware pose groups: {sorted(unknown)}")
    result = {}
    for group, positions in grouped_positions.items():
        if not isinstance(positions, dict):
            raise ValueError(f"{group} pose must be a JSON object")
        updates = (arm_positions(positions, mappings[group]) if group == "arm"
                   else CalibratedHandPose(mapping=mappings[group]).from_joint_degrees(positions) if group == "hand"
                   else mapped_positions(positions, mappings[group]))
        if set(result) & set(updates):
            raise ValueError("Multiple source entries map to the same model joint")
        result.update(updates)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "generated/humanoid_shadow.xml")
    poses = parser.add_mutually_exclusive_group()
    poses.add_argument("--pose", type=Path, help="JSON object: model joint names to radians")
    poses.add_argument("--real-arm-pose", type=Path, help="JSON object: hardware arm joint names to radians")
    poses.add_argument("--real-pose", type=Path, help="Grouped JSON: arm/neck radians, body/hand degrees")
    parser.add_argument("--hand-encoder-pose", type=Path,
                        help="JSON: left/right motor IDs and encoder radians; applies FT calibration once")
    parser.add_argument("--hand-calibration-report", type=Path, help="Save the YAML-derived hand zero and input conversion")
    parser.add_argument("--mapping", type=Path, default=ROOT / "configs/arm_mapping.json")
    for group in ("body", "neck", "hand"):
        parser.add_argument(f"--{group}-mapping", type=Path, default=ROOT / f"configs/{group}_mapping.json")
    parser.add_argument("--render", type=Path, help="Save a PNG instead of opening a window")
    args = parser.parse_args()
    report_path = args.model.parent / "model_report.json"
    report = {}
    if report_path.is_file():
        report = json.loads(report_path.read_text())
        if args.real_arm_pose and report.get("arm_mapping_sha256"):
            if hashlib.sha256(args.mapping.read_bytes()).hexdigest() != report["arm_mapping_sha256"]:
                raise ValueError("映射已变化，请先用同一份映射重新运行 build_mujoco_model.py。")
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    hand_calibration = CalibratedHandPose(mapping=json.loads(args.hand_mapping.read_text()))
    positions = hand_calibration.zero_model_pose()
    if args.pose:
        positions.update(json.loads(args.pose.read_text()))
    elif args.real_arm_pose:
        positions.update(arm_positions(json.loads(args.real_arm_pose.read_text()), json.loads(args.mapping.read_text())))
    elif args.real_pose:
        raw_pose = json.loads(args.real_pose.read_text())
        paths = {"arm": args.mapping,
                 **{g: getattr(args, g + "_mapping") for g in ("body", "neck", "hand")}}
        for group in raw_pose:
            if group not in paths:
                raise ValueError(f"Unknown hardware pose group: {group}")
            built = report.get("joint_mapping_groups", {}).get(group)
            if built and hashlib.sha256(paths[group].read_bytes()).hexdigest() != built["sha256"]:
                raise ValueError(f"{group} 映射已变化，请先用同一份映射重新生成模型。")
        positions.update(fullbody_positions(raw_pose, {g: json.loads(p.read_text()) for g, p in paths.items()}))
    calibration_report = hand_calibration.report()
    if args.hand_encoder_pose:
        explicit = (json.loads(args.pose.read_text()) if args.pose else {})
        if any(n in explicit for n in hand_calibration.zero_model_pose()) or (args.real_pose and raw_pose.get("hand")):
            parser.error("Choose either hand joint angles or encoder angles, not both")
        converted, degrees = hand_calibration.from_encoders(json.loads(args.hand_encoder_pose.read_text()))
        positions.update(converted)
        calibration_report["input_calibrated_joint_degrees"] = degrees
    if args.hand_calibration_report:
        args.hand_calibration_report.parent.mkdir(parents=True, exist_ok=True)
        args.hand_calibration_report.write_text(json.dumps(calibration_report, indent=2) + "\n")
    apply_pose(model, data, positions)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    camera = mujoco.MjvCamera()
    camera.lookat[:] = model.stat.center
    camera.distance = 1.8 * model.stat.extent
    camera.azimuth = 140
    camera.elevation = -15
    if args.render:
        from PIL import Image
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=option)
            args.render.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(renderer.render()).save(args.render)
        print(args.render.resolve())
        return
    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = camera.lookat
        viewer.cam.distance = camera.distance
        viewer.cam.azimuth = camera.azimuth
        viewer.cam.elevation = camera.elevation
        viewer.opt.geomgroup[3] = 0
        print("mujoco仿真已启动", flush=True)
        while viewer.is_running():
            # GUI changes are displayed kinematically; never integrate mj_step.
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
