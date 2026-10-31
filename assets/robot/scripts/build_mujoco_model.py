"""Convert the selected whole-body URDF to a MuJoCo shadow model.

Preserves URDF coordinates. The output is for qpos + mj_forward display;
it contains no actuators, robot contacts or calibrated dynamics settings.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from joint_calibration import audit_mapping
from model_paths import SOURCE_URDF


ROOT = Path(__file__).resolve().parents[1]
SOURCE = SOURCE_URDF
OUTPUT = ROOT / "generated"
MAPPING = ROOT / "configs/arm_mapping.json"
STL_DTYPE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
MAX_STL_FACES = 200_000


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_xml(root, destination):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(destination, encoding="utf-8", xml_declaration=True)


def load_stl(path):
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"Invalid binary STL: {path}")
    count = struct.unpack_from("<I", raw, 80)[0]
    if count == 0 or len(raw) != 84 + 50 * count:
        raise ValueError(f"Expected nonempty binary STL: {path}")
    triangles = np.frombuffer(raw, dtype=STL_DTYPE, count=count, offset=84)["vertices"]
    if not np.isfinite(triangles).all():
        raise ValueError(f"Non-finite STL vertices: {path}")
    return triangles


def prepare_mesh(path, output_dir, target_faces):
    triangles = load_stl(path)
    if len(triangles) <= MAX_STL_FACES:
        return path, None
    # Weld identical STL vertices before quadric mesh simplification.
    import fast_simplification

    points, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3).astype(np.int32)
    points_out, faces_out = fast_simplification.simplify(
        points.astype(np.float64), faces, target_count=target_faces, agg=7.0
    )
    reduced = points_out[faces_out].astype(np.float32)
    if not 0 < len(reduced) <= MAX_STL_FACES:
        raise ValueError(f"Simplification did not satisfy MuJoCo STL limit: {path}")
    old_bounds = np.stack((triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))))
    new_bounds = np.stack((reduced.min(axis=(0, 1)), reduced.max(axis=(0, 1))))
    bounds_error = float(np.max(np.abs(old_bounds - new_bounds)))
    if bounds_error > 0.005:
        raise ValueError(f"Mesh bounds changed by {bounds_error:.6g} m: {path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / f"{path.stem}_{sha256(path)[:12]}_{target_faces}.stl"
    records = np.zeros(len(reduced), dtype=STL_DTYPE)
    records["vertices"] = reduced
    normals = np.cross(reduced[:, 1] - reduced[:, 0], reduced[:, 2] - reduced[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    good = lengths > 0
    normals[good] /= lengths[good, None]
    records["normal"] = normals
    target.write_bytes(b"MuJoCo simplified mesh; source preserved".ljust(80, b"\0")
                       + struct.pack("<I", len(records)) + records.tobytes())
    return target, {"source": str(path), "source_sha256": sha256(path),
                    "output": str(target), "source_faces": len(triangles),
                    "output_faces": len(reduced), "bounds_error_m": bounds_error,
                    "method": "quadric_error_decimation", "aggression": 7.0,
                    "reference": "openarm_mission/sixrobot_sim/mesh_assets.py::_generate_visual"}


def origin_matrix(origin):
    result = np.eye(4)
    if origin is None:
        return result
    x, y, z = map(float, origin.get("rpy", "0 0 0").split())
    sx, cx, sy, cy, sz, cz = math.sin(x), math.cos(x), math.sin(y), math.cos(y), math.sin(z), math.cos(z)
    result[:3, :3] = (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
                       @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
                       @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))
    result[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
    return result


def reference_fk(robot, pose):
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    roots = {l.get("name") for l in robot.findall("link")} - children
    if len(roots) != 1:
        raise ValueError("URDF must have exactly one root")
    frames = {roots.pop(): np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        remaining = []
        for joint in pending:
            parent = joint.find("parent").get("link")
            if parent not in frames:
                remaining.append(joint)
                continue
            transform = origin_matrix(joint.find("origin"))
            kind = joint.get("type")
            if kind != "fixed":
                value = pose[joint.get("name")]
                axis = np.array(list(map(float, joint.find("axis").get("xyz").split())))
                axis /= np.linalg.norm(axis)
                if kind in ("revolute", "continuous"):
                    x, y, z = axis
                    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    rotation = np.eye(3) + math.sin(value) * skew + (1 - math.cos(value)) * (skew @ skew)
                    transform[:3, :3] = transform[:3, :3] @ rotation
                elif kind == "prismatic":
                    transform[:3, 3] += transform[:3, :3] @ (axis * value)
                else:
                    raise ValueError(f"Unsupported joint type: {kind}")
            frames[joint.find("child").get("link")] = frames[parent] @ transform
        if len(remaining) == len(pending):
            raise ValueError("URDF has a disconnected or cyclic joint graph")
        pending = remaining
    return frames


def verify_kinematics(robot, model):
    joints = {j.get("name"): j for j in robot.findall("joint") if j.get("type") != "fixed"}
    actual = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    if actual != set(joints) or model.nq != len(joints) or model.nu != 0:
        raise ValueError("Compiled shadow model joint set or dimensions differ from URDF")
    data = mujoco.MjData(model)
    max_position = max_rotation = max_limit = 0.0
    for fraction in (None, 0.25, 0.75):
        pose = {}
        for name, joint in joints.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            bounds = np.array([float(joint.find("limit").get(key)) for key in ("lower", "upper")])
            max_limit = max(max_limit, float(np.max(np.abs(model.jnt_range[jid] - bounds))))
            pose[name] = 0.0 if fraction is None else float(bounds[0] + fraction * (bounds[1] - bounds[0]))
            data.qpos[model.jnt_qposadr[jid]] = pose[name]
        mujoco.mj_forward(model, data)
        for name, transform in reference_fk(robot, pose).items():
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise ValueError(f"URDF link missing from MuJoCo bodies: {name}")
            max_position = max(max_position, float(np.linalg.norm(data.xpos[bid] - transform[:3, 3])))
            max_rotation = max(max_rotation, float(np.max(np.abs(data.xmat[bid].reshape(3, 3) - transform[:3, :3]))))
    # mj_saveLastXML uses finite decimal precision in the exported MJCF.
    if max(max_position, max_rotation, max_limit) > 2e-5:
        raise ValueError(f"URDF/MJCF parity failed: position={max_position}, rotation={max_rotation}, limit={max_limit}")
    return {"poses": 3, "links_per_pose": len(robot.findall("link")),
            "max_position_error_m": max_position, "max_rotation_matrix_error": max_rotation,
            "max_joint_limit_error_rad": max_limit, "tolerance": 2e-5}


def visual_minimum_z(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    minimum = float("inf")
    for gid in range(model.ngeom):
        if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_group[gid] == 3:
            continue
        mid = int(model.geom_dataid[gid])
        start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        vertices = model.mesh_vert[start:start + count]
        world = vertices @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
        minimum = min(minimum, float(world[:, 2].min()))
    if not math.isfinite(minimum):
        raise ValueError("No visual mesh vertices")
    return minimum


def build(source=SOURCE, output=OUTPUT, mapping_path=MAPPING, target_faces=150_000, group_mapping_paths=None):
    source, output, mapping_path = Path(source).resolve(), Path(output).resolve(), Path(mapping_path).resolve()
    if source.suffix != ".urdf":
        raise ValueError("Select the active .urdf, not a backup")
    if not 0 < target_faces <= MAX_STL_FACES:
        raise ValueError("target_faces must be within 1..200000")
    robot = ET.parse(source).getroot()
    original = ET.fromstring(ET.tostring(robot))
    mapping = json.loads(mapping_path.read_text())
    audit = []
    for spec in mapping["joints"]:
        if spec["sign"] not in (-1, 1) or not math.isfinite(spec["offset_rad"]):
            raise ValueError(f"Invalid arm mapping: {spec}")
        limit = robot.find(f"joint[@name='{spec['model_joint']}']/limit")
        if limit is None:
            raise ValueError(f"Arm joint missing: {spec['model_joint']}")
        actual = [float(limit.get(k)) for k in ("lower", "upper")]
        expected = sorted(spec["sign"] * v + spec["offset_rad"] for v in spec["real_limits_rad"])
        if not np.allclose(actual, expected, rtol=0, atol=1e-6):
            raise ValueError(f"Arm limit/mapping mismatch: {spec['model_joint']}: {actual} != {expected}")
        if not np.allclose(spec["model_limits_rad"], expected, rtol=0, atol=1e-6):
            raise ValueError(f"Stored model limits/mapping mismatch: {spec['model_joint']}")
        audit.append({**spec, "urdf_limits_rad": actual})

    group_mapping_paths = group_mapping_paths or {
        group: ROOT / f"configs/{group}_mapping.json" for group in ("body", "neck", "hand")
    }
    group_audits = {}
    mapping_groups = {"arm": {"file": str(mapping_path), "sha256": sha256(mapping_path),
        "zero_source": mapping.get("zero_source"), "physical_zero_verified": mapping["physical_zero_verified"],
        "joint_count": len(audit), "source_unit": "rad"}}
    for group in ("body", "neck", "hand"):
        path = Path(group_mapping_paths[group]).resolve()
        group_mapping = json.loads(path.read_text())
        group_audits[group] = audit_mapping(robot, group_mapping, group)
        mapping_groups[group] = {"file": str(path), "sha256": sha256(path),
            "zero_source": group_mapping.get("zero_source"),
            "physical_zero_verified": group_mapping["physical_zero_verified"],
            "joint_count": len(group_mapping["joints"]), "source_unit": group_mapping["source_unit"]}
    mapped_names = [s["model_joint"] for s in audit]
    mapped_names += [s["model_joint"] for rows in group_audits.values() for s in rows]
    urdf_names = {j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"}
    if len(mapped_names) != len(set(mapped_names)) or set(mapped_names) != urdf_names:
        raise ValueError("Joint mappings must cover every movable URDF joint exactly once")

    output.mkdir(parents=True, exist_ok=True)
    assets = {}
    reductions = []
    for link in robot.findall("link"):
        for kind in ("visual", "collision"):
            for index, geom in enumerate(link.findall(kind)):
                geom.set("name", f"{link.get('name')}__{kind}__{index}")
                mesh = geom.find("geometry/mesh")
                if mesh is None:
                    continue
                filename = mesh.get("filename")
                if filename.startswith("package://"):
                    raise ValueError("Resolve package:// assets before using this standalone converter")
                mesh_source = (source.parent / filename).resolve()
                if mesh_source not in assets:
                    target, reduction = prepare_mesh(mesh_source, output / "mesh_assets", target_faces)
                    assets[mesh_source] = target
                    if reduction:
                        reductions.append(reduction)
                mesh.set("filename", Path(os.path.relpath(assets[mesh_source], output)).as_posix())

    extension = robot.find("mujoco")
    if extension is None:
        extension = ET.SubElement(robot, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.attrib.update(angle="radian", strippath="false", discardvisual="false",
                          fusestatic="false", balanceinertia="true")
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    roots = {l.get("name") for l in robot.findall("link")} - children
    if len(roots) != 1:
        raise ValueError("Expected exactly one source root link")
    # A zero fixed wrapper retains the original root link as a named body.
    root_name = roots.pop()
    if robot.find("link[@name='shadow_world_anchor']") is not None:
        raise ValueError("Reserved wrapper name already present")
    ET.SubElement(robot, "link", name="shadow_world_anchor")
    joint = ET.SubElement(robot, "joint", name="shadow_world_to_source", type="fixed")
    ET.SubElement(joint, "parent", link="shadow_world_anchor")
    ET.SubElement(joint, "child", link=root_name)
    ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    wrapped = output / "humanoid_mujoco.urdf"
    write_xml(robot, wrapped)
    compiled = mujoco.MjModel.from_xml_path(str(wrapped))
    with tempfile.NamedTemporaryFile(suffix=".xml", dir=output) as stream:
        mujoco.mj_saveLastXML(stream.name, compiled)
        mjcf = ET.parse(stream.name).getroot()
    for mesh in mjcf.findall("asset/mesh"):
        if not (output / mesh.get("file")).resolve().is_file():
            raise FileNotFoundError(mesh.get("file"))
    for geom in mjcf.iter("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("group", "3" if "__collision__" in geom.get("name", "") else "0")
    option = mjcf.find("option")
    if option is None:
        option = ET.SubElement(mjcf, "option")
    option.set("gravity", "0 0 0")
    mjcf.set("model", "humanoid_shadow")
    destination = output / "humanoid_shadow.xml"
    write_xml(mjcf, destination)
    model = mujoco.MjModel.from_xml_path(str(destination))
    floor_z = visual_minimum_z(model) - 0.001
    world = mjcf.find("worldbody")
    ET.SubElement(world, "geom", name="display_floor", type="plane", size="2 2 0.05",
                  pos=f"0 0 {floor_z:.12g}", rgba="0.28 0.31 0.34 1", contype="0", conaffinity="0")
    ET.SubElement(world, "light", name="display_light", pos="0 -3 4", dir="0 1 -1",
                  diffuse="0.7 0.7 0.7")
    write_xml(mjcf, destination)
    model = mujoco.MjModel.from_xml_path(str(destination))
    verification = verify_kinematics(original, model)
    addresses = [{"name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i),
                  "joint_id": i, "qpos_address": int(model.jnt_qposadr[i]),
                  "dof_address": int(model.jnt_dofadr[i]), "range_rad": model.jnt_range[i].tolist()}
                 for i in range(model.njnt)]
    report = {"mode": "shadow_qpos_mj_forward", "source_urdf": str(source),
              "source_sha256": sha256(source), "mujoco_version": mujoco.__version__,
              "python_executable": sys.executable,
              "output_mjcf": str(destination), "output_sha256": sha256(destination),
              "arm_mapping": str(mapping_path),
              "arm_mapping_sha256": sha256(mapping_path),
              "arm_zero_source": mapping.get("zero_source", mapping.get("offset_basis")),
              "joint_mapping_groups": mapping_groups,
              "group_limits": group_audits,
              "physical_arm_zero_verified": mapping["physical_zero_verified"],
              "nq": model.nq, "nv": model.nv, "nu": model.nu, "nbody": model.nbody,
              "nmesh": model.nmesh, "ngeom": model.ngeom,
              "robot_world_translation_m": [0, 0, 0], "display_floor_z_m": floor_z,
              "compiler_balanceinertia": True,
              "dynamics_validated": False, "contacts_enabled": False,
              "simplified_meshes": reductions, "kinematic_verification": verification,
              "arm_limits": audit, "joints": addresses}
    (output / "model_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"model": str(destination), "nq": model.nq, "nu": model.nu,
                      "simplified_meshes": len(reductions), "verification": verification}, indent=2))
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--mapping", type=Path, default=MAPPING)
    parser.add_argument("--target-faces", type=int, default=150_000)
    for group in ("body", "neck", "hand"):
        parser.add_argument(f"--{group}-mapping", type=Path, default=ROOT / f"configs/{group}_mapping.json")
    args = parser.parse_args()
    build(args.source, args.output, args.mapping, args.target_faces,
          {group: getattr(args, group + "_mapping") for group in ("body", "neck", "hand")})
