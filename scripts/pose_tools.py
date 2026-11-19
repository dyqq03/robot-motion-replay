"""Reusable model-space posture measurements and bounded, collision-screened IK seeds."""
import argparse,os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import numpy as np
from common import ROBOT, SKILL, read, write, digest


def unit(v):
    v=np.asarray(v,float);length=np.linalg.norm(v)
    if not np.isfinite(v).all() or length<1e-9:raise ValueError('Nonzero finite direction required')
    return v/length


def angle(a,b):return float(np.rad2deg(np.arccos(np.clip(np.dot(unit(a),unit(b)),-1,1))))


def measurements(model,data,side):
    cfg=read(SKILL/'assets/posture_landmarks.json')
    ids={k:model.body(v.format(Side=side.title())).id for k,v in cfg['body_names'].items()}
    points={k:data.xpos[v].copy() for k,v in ids.items()}
    normal=data.xmat[ids['palm']].reshape(3,3)@unit(cfg['palm_normal_local_axis'])
    forearm=unit(points['wrist']-points['elbow'])
    hand=unit(points['middle_base']-points['palm'])
    return dict(side=side,positions_m={k:v.tolist() for k,v in points.items()},
        palm_normal_world=normal.tolist(),palm_normal_down_deg=float(np.rad2deg(np.arcsin(np.clip(-normal[2],-1,1)))),
        forearm_direction_world=forearm.tolist(),palm_to_middle_base_direction_world=hand.tolist(),
        forearm_to_middle_base_deg=angle(forearm,hand),elbow_height_m=float(points['elbow'][2]),
        landmark_sha256=digest(SKILL/'assets/posture_landmarks.json'),interpretation=cfg['interpretation'])


def posture_frame(model,data,side,title):
    import mujoco
    from PIL import Image,ImageDraw,ImageFont
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    image=Image.new('RGB',(1280,512),'#171b22')
    cameras=[]
    for az in (165.,75.):
        camera=mujoco.MjvCamera();camera.azimuth=az;camera.elevation=-8;camera.distance=1.85;camera.lookat[:]=[.08,0,1.14];cameras.append(camera)
    with mujoco.Renderer(model,height=480,width=480) as renderer:
        for i,camera in enumerate(cameras):
            renderer.update_scene(data,camera=camera,scene_option=opt)
            image.paste(Image.fromarray(renderer.render()),(i*480,32))
    with mujoco.Renderer(model,height=240,width=320) as renderer:
        for i,hand in enumerate(('left','right')):
            camera=mujoco.MjvCamera();camera.azimuth=130 if hand=='left' else 200;camera.elevation=-10;camera.distance=.45
            camera.lookat[:]=data.xpos[model.body(hand.title()+'Palm').id]+[.045,0,-.025]
            renderer.update_scene(data,camera=camera,scene_option=opt);image.paste(Image.fromarray(renderer.render()),(960,32+i*240))
    font=ImageFont.truetype(str(SKILL/'assets/fonts/NotoSansCJK-Regular.ttc'),18)
    m=measurements(model,data,side)
    label=f"{title} | elbow z={m['elbow_height_m']:.3f}m | palm down={m['palm_normal_down_deg']:.1f}deg | forearm/palm={m['forearm_to_middle_base_deg']:.1f}deg"
    ImageDraw.Draw(image).text((8,5),label,font=font,fill='white')
    return image


def measure_package(directory,seconds,side,image=None):
    import mujoco
    from preview_motion_package import load_package
    from motion_collision import model_trajectory
    model,data,base,t,poses,meta=load_package(Path(directory),check_collision_provenance=False)
    if not np.isfinite(seconds) or not 0<=seconds<=t[-1]:raise ValueError('Time outside package clock')
    path=model_trajectory(model,base,poses)
    data.qpos[:]=[np.interp(seconds,t,path[:,j]) for j in range(model.nq)]
    mujoco.mj_forward(model,data)
    if image:
        image=Path(image);image.parent.mkdir(parents=True,exist_ok=True)
        posture_frame(model,data,side,f'{side} {seconds:.3f}s').save(image)
    return dict(time_s=seconds,measurements=measurements(model,data,side),model_sha256=digest(ROBOT/'generated/humanoid_shadow.xml'))


def solve(request,image=None):
    import mujoco
    from scipy.optimize import least_squares
    from motion import designer
    from collision_checker import CollisionChecker
    from doc50_kinematics import ARM_NAMES
    d=designer();side=request['side']
    if side not in ('left','right'):raise ValueError('side must be left/right')
    off=0 if side=='left' else 6;seed=np.asarray(request.get('seed_deg',d.rest[off:off+6]),float)
    if seed.shape!=(6,) or not np.isfinite(seed).all():raise ValueError('Six finite seed angles required')
    specs={r['source_joint']:r for r in d.mapping['joints']}
    bounds=np.rad2deg(np.array([specs[n]['real_limits_rad'] for n in ARM_NAMES[off:off+6]]))
    lo,hi=bounds[:,0]+1e-5,bounds[:,1]-1e-5
    j7=float(request.get('j7_target_deg',seed[5]));band=float(request.get('j7_tolerance_deg',2.))
    if not np.isfinite([j7,band]).all() or band<=0:raise ValueError('Positive finite J7 band required')
    lo[5]=max(lo[5],j7-band);hi[5]=min(hi[5],j7+band)
    if np.any(lo>=hi):raise ValueError('Requested posture bounds do not intersect joint limits')
    target=np.asarray(request['palm_position_m'],float)
    if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Finite xyz target required')
    normal=unit(request['palm_normal_world']) if 'palm_normal_world' in request else None
    ceiling=request.get('elbow_max_z_m')
    if ceiling is not None and not np.isfinite(ceiling):raise ValueError('Finite elbow ceiling required')
    checker=CollisionChecker(d.model,ROBOT/'configs/design_collision_rules.json')
    def apply(q):
        full=d.rest.copy();full[off:off+6]=q
        d.data.qpos[:]=d.model_path(full[None,:])[0];mujoco.mj_forward(d.model,d.data)
        return measurements(d.model,d.data,side)
    def residual(q):
        m=apply(q);parts=[(np.array(m['positions_m']['palm'])-target)*10,(q-seed)*.001,np.array([(q[5]-j7)*.03])]
        if normal is not None:parts.append((np.array(m['palm_normal_world'])-normal)*.8)
        if ceiling is not None:parts.append(np.array([max(0,m['elbow_height_m']-ceiling)*20]))
        return np.concatenate(parts)
    starts=[seed,seed+np.array([5,5,0,-10,0,0]),seed+np.array([-5,-5,0,10,0,0])]
    rows=[]
    for start in starts:
        fit=least_squares(residual,np.clip(start,lo+1e-8,hi-1e-8),bounds=(lo,hi),max_nfev=180)
        m=apply(fit.x);quick=checker.quick(d.data)
        candidate=checker.candidate(d.data)
        rows.append(dict(angles_deg=fit.x.tolist(),score=float(np.linalg.norm(fit.fun)),measurements=m,
            position_error_m=float(np.linalg.norm(np.array(m['positions_m']['palm'])-target)),
            quick=quick,candidate=candidate,can_export=False))
    rows.sort(key=lambda r:r['score'])
    if image:
        from PIL import Image
        sheet=Image.new('RGB',(1280,512*len(rows)))
        for i,row in enumerate(rows):
            apply(np.asarray(row['angles_deg']))
            sheet.paste(posture_frame(d.model,d.data,side,f'candidate {i+1}, position error {row["position_error_m"]:.4f}m'),(0,i*512))
        image=Path(image);image.parent.mkdir(parents=True,exist_ok=True);sheet.save(image)
    return dict(request=request,candidates=rows,scope='Posture seeds only. Inspect images, enforce semantic targets, then check the complete trajectory.',
        model_sha256=digest(ROBOT/'generated/humanoid_shadow.xml'),hardware_commands_sent=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
    x=sub.add_parser('measure');x.add_argument('package',type=Path);x.add_argument('--time',type=float,required=True);x.add_argument('--side',choices=['left','right'],required=True);x.add_argument('--out',type=Path,required=True);x.add_argument('--image',type=Path)
    x=sub.add_parser('solve');x.add_argument('request',type=Path);x.add_argument('--out',type=Path,required=True);x.add_argument('--image',type=Path)
    a=p.parse_args();result=measure_package(a.package,a.time,a.side,a.image) if a.cmd=='measure' else solve(read(a.request),a.image);write(a.out,result)
