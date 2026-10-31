"""Read downloaded arm recordings for local viewing without rewriting source data."""
from pathlib import Path
import hashlib
import json

import h5py
import mujoco
import numpy as np

from view_shadow_model import apply_pose, arm_positions
from calibrated_hand_pose import calibrated_hand_zero_pose

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f"openarm_{side}_joint{j}" for side in ("left", "right") for j in (1, 2, 3, 4, 5, 7)]


def load_recording(directory):
    directory = Path(directory).resolve()
    download = json.loads((directory.parent / "download_manifest.json").read_text())
    verified = []
    for item in download["verified_files"]:
        path = directory.parent / item["path"]
        if path.parent != directory:
            continue
        if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Downloaded recording checksum mismatch: " + str(path))
        verified.append(path.name)
    if "fullbody.hdf5" not in verified or len(verified) != 4:
        raise ValueError("Expected a complete checksum-verified server recording")
    meta = json.loads((directory / "fullbody_meta.json").read_text())
    if meta.get("missing_joints"):
        raise ValueError("Recording has missing joints")
    streams = []
    with h5py.File(directory / "fullbody.hdf5") as f:
        others = {key: len(f[key+"/q"]) if key+"/q" in f else 0
                  for key in ("neck", "eyou", "hand", "hand_left", "hand_right")}
        if any(others.values()):
            raise ValueError("This recorded viewer supports arm-only recordings; other joint streams are present")
        if list(f["arms"].attrs["joint_names"]) != NAMES:
            raise ValueError("Unexpected recorded joint order")
        mode = str(f.attrs["arm_mode"])
        if mode in ("ker", "mixed") and len(f["arms/cmd_left"]):
            for side, suffix, names in (("left", "l", NAMES[:6]), ("right", "r", NAMES[6:])):
                streams.append((names, f["arms/t_arm_cmd_"+suffix][:], f["arms/cmd_"+side][:]))
            selected = "arm_commands"
        elif mode in ("teach", "mixed") and len(f["arms/joint_position"]):
            streams.append((NAMES, f["arms/t_arm_js"][:], f["arms/joint_position"][:]))
            selected = "measured_arm_positions_teach_mode"
        else:
            raise ValueError("Recording has no usable arm stream")
    for names, ts, q in streams:
        if (len(ts)<2 or q.shape!=(len(ts), len(names)) or not np.isfinite(ts).all()
                or not np.isfinite(q).all() or ts[0]<0 or np.any(np.diff(ts)<=0)):
            raise ValueError("Invalid original arm stream")
    # Retain both independent timelines: their union preserves every recorded knot.
    # Before a stream's first sample, show its first pose (the server approach target).
    t = np.unique(np.concatenate([np.array([0.])] + [s[1] for s in streams]))
    raw = np.empty((len(t), 12))
    for names, ts, q in streams:
        for column, name in enumerate(names):
            raw[:, NAMES.index(name)] = np.interp(t, ts, q[:,column])
    mapping = json.loads((ROOT / "configs/arm_mapping.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(ROOT / "generated/humanoid_shadow.xml"))
    data = mujoco.MjData(model)
    base = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): 0. for i in range(model.njnt)}
    base.update(calibrated_hand_zero_pose())
    base.update(arm_positions(dict(zip(NAMES, raw[0])), mapping))
    poses = [arm_positions(dict(zip(NAMES, row)), mapping) for row in raw]
    for pose in poses:
        apply_pose(model, data, base | pose)
    motion = dict(name="服务器原始录制 " + directory.name, joint_names=NAMES,
                  preview_label="SERVER ORIGINAL: " + directory.name, duration_s=float(t[-1]),
                  sample_hz=None, preview_times_s=np.linspace(0, t[-1], 6).tolist(),
                  source="downloaded_server_recording", arm_mode=mode, selected_stream=selected,
                  verified_source_files=verified, other_stream_frames=others,
                  original_streams=[dict(names=ns, samples=len(ts), first_s=float(ts[0]), last_s=float(ts[-1]))
                                    for ns, ts, _ in streams],
                  source_hdf5_sha256=hashlib.sha256((directory / "fullbody.hdf5").read_bytes()).hexdigest(),
                  loop_endpoints_match=bool(np.allclose(raw[0], raw[-1], rtol=0, atol=1e-10)),
                  before_first_sample="hold_first_recorded_pose", synthetic=False)
    return model, data, base, t, poses, motion
