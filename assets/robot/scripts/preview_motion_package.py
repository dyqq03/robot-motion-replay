"""Play a generated command package in the local MuJoCo shadow; no network or ROS."""
from __future__ import annotations

import argparse
from functools import partial
import hashlib
import json
from pathlib import Path
import subprocess
import shutil
import time

import h5py
import mujoco
import numpy as np

from view_shadow_model import apply_pose, arm_positions
from calibrated_hand_pose import CalibratedHandPose, calibrated_hand_zero_pose
from model_paths import model_reference_point

ROOT = Path(__file__).resolve().parents[1]


def load_package(directory, *, check_collision_provenance=True, collision_rules_path=None, model=None):
    directory=Path(directory)
    manifest=json.loads((directory/'manifest.json').read_text())
    for filename,expected in manifest['files'].items():
        if Path(filename).name!=filename:raise ValueError('Invalid manifest path')
        if hashlib.sha256((directory/filename).read_bytes()).hexdigest()!=expected:
            raise ValueError('Package checksum mismatch: '+filename)
    if check_collision_provenance and manifest.get('collision_check_required'):
        from motion_collision import validate_package_report
        validate_package_report(directory,manifest,rules_path=collision_rules_path)
    model_path=ROOT/'generated/humanoid_shadow.xml'
    mapping_path=ROOT/'configs/arm_mapping.json'
    for path,key in ((model_path,'model_sha256'),(mapping_path,'arm_mapping_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=manifest[key]:
            raise ValueError('Package calibration/model mismatch: '+str(path))
    mapping=json.loads(mapping_path.read_text())
    motion=json.loads((directory/'motion.json').read_text())
    sides=('left','right')
    arm_names={s:[f'openarm_{s}_joint{j}' for j in (1,2,3,4,5,7)] for s in sides}
    names=motion.get('joint_names',[])
    if names not in ([],arm_names['left'],arm_names['right'],arm_names['left']+arm_names['right']):
        raise ValueError('Expected named single or bilateral arm commands')
    hand_names=motion.get('hand_joint_names',{})
    # New packages snapshot their absolute measured reference; historical
    # packages without this field keep the original calibrated-zero contract.
    hand_rest=motion.get('hand_baseline_pose_deg')
    hand_cal=None
    if hand_names or hand_rest is not None:
        if hand_rest is not None and (not isinstance(hand_rest,dict) or set(hand_rest)-set(sides)):
            raise ValueError('Invalid named hand reference sides')
        if not isinstance(hand_names,dict) or set(hand_names)-set(sides):raise ValueError('Invalid hand sides')
        hand_cal=CalibratedHandPose()
        for filename,key in (('hand_mapping.json','hand_mapping_sha256'),('hand_hardware.json','hand_hardware_sha256')):
            if hashlib.sha256((ROOT/'configs'/filename).read_bytes()).hexdigest()!=manifest.get(key):
                raise ValueError('Package hand calibration mismatch: '+filename)
        if manifest.get('hand_calibration_revision')!=hand_cal.revision:raise ValueError('Package hand calibration revision mismatch')
        for side,ns in hand_names.items():
            expected=hand_cal.models[side+'_hand'].names
            if len(ns)!=16 or len(set(ns))!=16 or set(ns)!=set(expected):raise ValueError('Expected all 16 named calibrated joints per hand')
        if hand_rest is not None:
            if set(hand_names)-set(hand_rest):raise ValueError('Missing commanded hand reference')
            for side,reference in hand_rest.items():
                if not isinstance(reference,dict) or set(reference)!=set(hand_cal.models[side+'_hand'].names):
                    raise ValueError('Expected all 16 named hand reference angles')
                hand_cal.from_joint_degrees({side+'/'+n:v for n,v in reference.items()})
    neck_names=motion.get('neck_joint_names',[])
    neck_mapping=None
    if neck_names:
        if neck_names!=['neck_joint1','neck_joint2']:raise ValueError('Expected named neck J1/J2 commands')
        neck_path=ROOT/'configs/neck_mapping.json'
        if hashlib.sha256(neck_path.read_bytes()).hexdigest()!=manifest.get('neck_mapping_sha256'):raise ValueError('Package neck calibration mismatch')
        neck_mapping=json.loads(neck_path.read_text())
    streams=[]
    with h5py.File(directory/'fullbody.hdf5') as f:
        if any(len(f[key]) for key in ('arms/joint_position','arms/t_arm_js','eyou/q','eyou/t','hand/q','hand/t')):
            raise ValueError('Unexpected command or feedback stream')
        for side,suffix in (('left','l'),('right','r')):
            ts=f['arms/t_arm_cmd_'+suffix][:];q=f['arms/cmd_'+side][:]
            if not names:
                if len(ts) or len(q):raise ValueError('Unexpected arm command stream')
                continue
            if q.shape!=(len(ts),6):raise ValueError('Arm command dimensions do not match timestamps')
            selected=any(n in names for n in arm_names[side])
            if not selected:
                held=[motion['preview_arm_baseline_rad'][n] for n in arm_names[side]]
                if not np.allclose(q,held,rtol=0,atol=1e-12):raise ValueError('Inactive arm must hold its reference pose')
            streams.append(('arms',ts,q,arm_names[side]))
        for side in sides:
            g=f['hand_'+side];q=g['q'][:];ts=g['t'][:]
            if side not in hand_names:
                if len(q) or len(ts):raise ValueError('Unexpected hand command stream')
                continue
            ns=[n.decode() if isinstance(n,bytes) else str(n) for n in g.attrs.get('joint_names',[])]
            if ns!=hand_names[side] or g.attrs.get('units')!='deg' or q.shape!=(len(ts),16):
                raise ValueError('Invalid synchronized hand trajectory in calibrated degrees')
            reference=[hand_rest[side][n] for n in ns] if hand_rest is not None else np.zeros(16)
            if not np.allclose(q[[0,-1]],reference,rtol=0,atol=1e-10):raise ValueError('Hands must start and end at the declared hand reference (legacy default: calibrated zero)')
            streams.append((side,ts,q,ns))
        g=f['neck'];q=g['q'][:];ts=g['t'][:]
        if neck_names:
            if q.shape!=(len(ts),2) or list(g.attrs.get('joint_names',[]))!=neck_names:raise ValueError('Invalid synchronized neck trajectory')
            rest=motion.get('preview_neck_baseline_rad',{})
            if set(rest)!=set(neck_names):raise ValueError('Missing neck preview baseline')
            if not np.allclose(q[[0,-1]],[rest[n] for n in neck_names],rtol=0,atol=1e-12):raise ValueError('Neck loop endpoints do not match baseline')
            streams.append(('neck',ts,q,neck_names))
        elif len(q) or len(ts):raise ValueError('Unexpected neck command stream')
    if not streams:raise ValueError('No active command stream')
    t=streams[0][1]
    if len(t)<2 or not np.isfinite(t).all() or t[0]!=0 or np.any(np.diff(t)<=0) or abs(t[-1]-motion['duration_s'])>1e-8:
        raise ValueError('Invalid trajectory')
    for group,ts,q,ns in streams:
        if not np.array_equal(ts,t) or not np.isfinite(q).all():raise ValueError('Invalid synchronized hand trajectory' if group in ('left','right') else 'Invalid synchronized trajectory')
        if not np.allclose(q[0],q[-1],atol=1e-10):raise ValueError('Loop endpoints do not match')
    model=model or mujoco.MjModel.from_xml_path(str(model_path))
    data=mujoco.MjData(model)
    base={model.joint(i).name:0. for i in range(model.njnt)}
    base.update(calibrated_hand_zero_pose())
    if hand_rest is not None:
        base.update(hand_cal.from_joint_degrees({side+'/'+n:v for side,reference in hand_rest.items() for n,v in reference.items()}))
    base.update(arm_positions(motion['preview_arm_baseline_rad'],mapping))
    # New optional-chain packages explicitly declare the assumed uncommanded neck.
    neck_rest=motion.get('preview_neck_baseline_rad')
    if neck_rest:
        nm=neck_mapping or json.loads((ROOT/'configs/neck_mapping.json').read_text())
        base.update(arm_positions(neck_rest,nm))
    mapped=[{} for _ in t]
    for group,ts,q,ns in streams:
        for pose,row in zip(mapped,q):
            if group=='arms':pose.update(arm_positions(dict(zip(ns,row)),mapping))
            elif group=='neck':pose.update(arm_positions(dict(zip(ns,row)),neck_mapping))
            else:pose.update(hand_cal.from_joint_degrees({group+'/'+n:float(v) for n,v in zip(ns,row)}))
    for pose in mapped:apply_pose(model,data,base|pose)
    return model,data,base,t,mapped,motion

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    parser.add_argument("--recorded", action="store_true", help="View checksum-verified downloaded server recordings")
    parser.add_argument("--render-video", type=Path)
    parser.add_argument("--contact-sheet", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--collision-rules", type=Path, help="Rules used to verify a package with action-specific collision policy")
    parser.add_argument("--azimuth", type=float, default=165.)
    parser.add_argument("--distance", type=float, default=2.55, help="Camera distance in meters")
    parser.add_argument("--lookat", nargs=3, type=float, default=None, help="Camera target XYZ in current world coordinates")
    parser.add_argument("--elevation", type=float, default=-8.)
    parser.add_argument("--then", nargs="+", type=Path, default=[], help="Additional packages in the GUI playback loop")
    parser.add_argument("--cycles", type=int, default=None, help="GUI cycles; default from package or zero (loop until closed)")
    args = parser.parse_args()
    if args.then and (args.render_video or args.contact_sheet):
        parser.error("--then is available for GUI playback only")
    if args.recorded:
        from recorded_motion import load_recording
        loader = load_recording
    else:
        loader = partial(load_package, collision_rules_path=args.collision_rules)
        for path in [args.package, *args.then]:
            manifest = json.loads((path / "manifest.json").read_text())
            if manifest.get("draft"):
                print(f"草稿预览：{path.name}，尚未通过完整碰撞检查。", flush=True)
    model, data, baseline, t, poses, motion = loader(args.package.resolve())
    if not args.recorded:
        loader = partial(load_package, collision_rules_path=args.collision_rules, model=model)
    names = list(poses[0])
    values = np.array([[p[n] for n in names] for p in poses])
    playlist = [(baseline, t, names, values, motion)]
    for path in args.then:
        _, _, b, ts, ps, meta = loader(path.resolve())
        ns = list(ps[0])
        playlist.append((b, ts, ns, np.array([[p[n] for n in ns] for p in ps]), meta))
    cycles=args.cycles
    if cycles is None:
        cycles=motion.get("preview_cycles",0) if len(playlist)==1 else 0
    if not isinstance(cycles,int) or cycles<0:
        parser.error("--cycles or package preview_cycles must be a nonnegative integer")
    camera = mujoco.MjvCamera()
    camera.lookat[:] = args.lookat if args.lookat is not None else model_reference_point([0., 0., .86])
    camera.distance, camera.azimuth, camera.elevation = args.distance, args.azimuth, args.elevation
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0

    def pose_at(seconds, entry=0):
        b, ts, ns, vs, _ = playlist[entry]
        pose = {n: float(np.interp(seconds, ts, vs[:, i])) for i, n in enumerate(ns)}
        apply_pose(model, data, b | pose)

    frames = 0
    if args.render_video or args.contact_sheet:
        from PIL import Image, ImageDraw
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            def render(seconds):
                pose_at(seconds)
                renderer.update_scene(data, camera=camera, scene_option=option)
                return renderer.render()

            if args.render_video:
                args.render_video.parent.mkdir(parents=True, exist_ok=True)
                executable = shutil.which("ffmpeg")
                if executable is None:
                    import imageio_ffmpeg
                    executable = imageio_ffmpeg.get_ffmpeg_exe()
                command = [executable, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", "640x480", "-r", "30", "-i", "pipe:0", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", "-crf", "20", "-movflags", "+faststart", str(args.render_video)]
                process = subprocess.Popen(command, stdin=subprocess.PIPE)
                try:
                    for seconds in np.r_[np.arange(0, t[-1], 1 / 30), t[-1]]:
                        process.stdin.write(render(seconds).tobytes())
                        frames += 1
                finally:
                    process.stdin.close()
                if process.wait() != 0:
                    raise RuntimeError("Video encoding failed")
            if args.contact_sheet:
                sheet = Image.new("RGB", (1920, 960))
                sample_times = motion.get("preview_times_s", (0., 2.2, 4.2, 4.65, 5.55, 13.2))
                for i, seconds in enumerate(sample_times):
                    im = Image.fromarray(render(seconds))
                    label = motion.get("preview_label", "RIGHT GREETING")
                    ImageDraw.Draw(im).text((16, 16), f"{label} | {seconds:.2f}s | LOCAL SHADOW", fill="white")
                    sheet.paste(im, ((i % 3) * 640, (i // 3) * 480))
                args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
                sheet.save(args.contact_sheet)
    else:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = camera.distance, camera.azimuth, camera.elevation
            viewer.opt.geomgroup[3] = 0
            print("本地影子模型已加载：" + " → ".join(p[4]['name'] for p in playlist), flush=True)
            start = time.monotonic()
            lengths = [p[1][-1] + 2 for p in playlist]
            total = sum(lengths)
            last_entry = None
            while viewer.is_running():
                elapsed = time.monotonic() - start
                cycle = int(elapsed // total)
                if cycles and cycle >= cycles:
                    break
                phase = elapsed % total
                entry = 0
                while phase >= lengths[entry] and entry < len(playlist)-1:
                    phase -= lengths[entry]
                    entry += 1
                if last_entry != entry:
                    print("正在播放：" + playlist[entry][4]['name'], flush=True)
                    last_entry = entry
                with viewer.lock():
                    pose_at(min(phase, playlist[entry][1][-1]), entry)
                viewer.sync()
                frames += 1
                time.sleep(1 / 60)
    if args.report:
        report = dict(package=str(args.package.resolve()), frames_rendered=frames, validated_samples=len(t),
                      preview_cycles=cycles,
                      duration_s=float(t[-1]), controlled_model_joints=names, other_joints_constant=True,
                      joint_limits_valid=True, loop_endpoints_match=motion.get("loop_endpoints_match", True), no_network_or_ros=True,
                      source=motion.get("source", "generated_package"),
                      source_hdf5_sha256=motion.get("source_hdf5_sha256"),
                      recorded_streams=motion.get("original_streams"),
                      hardware_execution_validated=False,
                      peak_speed_deg_s=motion.get("peak_speed_deg_s", motion.get("analytical_peak_speed_deg_s")))
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
