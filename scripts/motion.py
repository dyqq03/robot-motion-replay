"""Portable local authoring, simulation review, validation and export CLI."""
import argparse,json,math,re,shutil,sys,zipfile,os
from pathlib import Path
if len(sys.argv)>1 and sys.argv[1] in ['doctor','create','check','render']:os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np,h5py
from scipy.interpolate import make_interp_spline
from common import SKILL,ROBOT,WORK,read,write,digest,inputs,refresh_manifest
from doc50_kinematics import Designer,ARM_NAMES,NECK_NAMES
from motion_baseline import hand_baseline_deg

def designer():return Designer(hand_reference=hand_baseline_deg())
def identifier(name):
 if not re.fullmatch(r'fb_[A-Za-z0-9_]+',name):raise ValueError('Use a new fb_... versioned package name')
 return name
def track_spec(d,key):
 if key in ['arm_left','arm_right']:
  off=0 if key=='arm_left' else 6;return off,ARM_NAMES[off:off+6]
 if key=='neck':return 12,NECK_NAMES
 if key in ['hand_left','hand_right']:
  side=key[5:];return (14 if side=='left' else 30),d.hand_names[side]
 raise ValueError('Unsupported control chain: '+key)

def create(spec_path,out=None):
 from package_io import write_draft
 spec=read(spec_path);name=identifier(spec['package']);duration=float(spec['duration_s']);hz=float(spec.get('sample_hz',100))
 if not math.isfinite(duration) or not .2<=duration<=300 or not 20<=hz<=250:raise ValueError('Invalid duration/sample rate')
 d=designer();t=np.linspace(0,duration,int(math.ceil(duration*hz))+1);q=np.tile(d.rest,(len(t),1));arms=[];hands=[]
 if not spec.get('tracks'):raise ValueError('At least one command chain is required')
 modes={}
 for key,track in spec['tracks'].items():
  off,names=track_spec(d,key);mode=track['mode']
  if mode not in ['rest_relative','absolute']:raise ValueError('Declare track mode: rest_relative or absolute')
  frames=track['keyframes'];kt=np.array([f['t'] for f in frames],float)
  if len(kt)<2 or not np.isfinite(kt).all() or kt[0]!=0 or abs(kt[-1]-duration)>1e-9 or np.any(np.diff(kt)<=0):raise ValueError('Keyframes must cover 0..duration with increasing finite times')
  values=[]
  for frame in frames:
   row=np.zeros(len(names)) if mode=='rest_relative' else d.rest[off:off+len(names)].copy()
   if not isinstance(frame['q'],dict) or set(frame['q'])-set(names):raise ValueError('Use named joint angles for '+key)
   for n,v in frame['q'].items():
    if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v):raise ValueError('Finite joint degrees required')
    row[names.index(n)]=v
   values.append(row)
  # One global spline: intermediate keyframes do not implicitly stop.
  z=np.zeros(len(names));spline=make_interp_spline(kt,np.array(values),k=5,bc_type=([(1,z),(2,z)],[(1,z),(2,z)]))
  result=spline(t)
  if mode=='rest_relative':result+=d.rest[off:off+len(names)]
  q[:,off:off+len(names)]=result
  if key.startswith('arm_'):arms.append(key[4:])
  if key.startswith('hand_'):hands.append(key[5:]);modes[key[5:]]=mode
 caps={'arms':15. if spec.get('near_head') else 25.,'neck':15.,'hand_left':20.,'hand_right':20.}
 if set(spec.get('speed_caps_deg_s',{}))-set(caps):raise ValueError('Unknown speed cap group')
 caps.update(spec.get('speed_caps_deg_s',{}))
 if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in caps.values()):raise ValueError('Positive finite speed caps required')
 groupcols={'arms':slice(0,12),'neck':slice(12,14),'hand_left':slice(14,30),'hand_right':slice(30,46)}
 velocity=np.gradient(q,t,axis=0,edge_order=2)
 factor=max([1.]+[float(abs(velocity[:,sl]).max())/caps[g] for g,sl in groupcols.items()])
 if factor>1:
  if spec.get('extend_for_speed_limits',True):t*=factor*1.00001
  else:raise ValueError(f'Speed cap exceeded: extend duration by {factor:g}')
 meta=dict(index=spec.get('index',1),name=spec['name'],slug=name,source_page=None,source_excerpt=spec.get('description',spec['name']),source=spec.get('source','user_requested_design'),arms=arms,neck='neck' in spec['tracks'],hands=hands,near_head=bool(spec.get('near_head')),speed_caps=caps,preview_times_s=np.linspace(0,float(t[-1]),6).tolist(),hand_retarget_modes=modes,hand_endpoint_blend_fraction=.22,requested_duration_s=duration,automatic_duration_factor=float(t[-1]/duration),design_notes=spec.get('notes',[]))
 dest=Path(out) if out else WORK/'drafts'/name
 if dest.name!=name:raise ValueError('Output directory basename must equal package name')
 dest.parent.mkdir(parents=True,exist_ok=True)
 write_draft(dest,d,t,q,meta);write(dest/'design.json',spec);refresh_manifest(dest)
 print(json.dumps(dict(draft=str(dest),duration_s=float(t[-1]),automatic_duration_factor=meta['automatic_duration_factor']),ensure_ascii=False))
 return dest

def kinematics(directory):
 """Validate actual streams without requiring today's reference for historical examples."""
 from preview_motion_package import load_package
 from motion_collision import model_trajectory
 directory=Path(directory);m,d,base,t,poses,meta=load_package(directory,check_collision_provenance=False)
 specs={s['source_joint']:s for f in ['arm_mapping.json','neck_mapping.json','hand_mapping.json'] for s in read(ROBOT/'configs'/f)['joints']}
 speeds={};cols=[];arrays=[]
 with h5py.File(directory/'fullbody.hdf5') as f:
  paths=[('arms/cmd_left','arms/t_arm_cmd_l',ARM_NAMES[:6],'arms',True),('arms/cmd_right','arms/t_arm_cmd_r',ARM_NAMES[6:],'arms',True),('neck/q','neck/t',NECK_NAMES,'neck',True)]
  for side in ['left','right']:paths.append(('hand_'+side+'/q','hand_'+side+'/t',[side+'/'+n for n in meta.get('hand_joint_names',{}).get(side,[])],'hand_'+side,False))
  for key,tk,names,g,rad in paths:
   q=f[key][:];ts=f[tk][:]
   if not len(q):continue
   if not np.isfinite(q).all() or not np.isfinite(ts).all() or len(ts)<3 or np.any(np.diff(ts)<=0) or ts[0]!=0:raise ValueError('Invalid timeline: '+key)
   if abs(ts[-1]-t[-1])>1e-8:raise ValueError('Command chain end times must agree')
   values=np.rad2deg(q) if rad else q
   np.testing.assert_allclose(values[0],values[-1],rtol=0,atol=1e-8)
   if rad:
    reference=meta['preview_neck_baseline_rad'] if g=='neck' else meta['preview_arm_baseline_rad']
    np.testing.assert_allclose(q[[0,-1]],np.tile([reference[n] for n in names],(2,1)),rtol=0,atol=1e-8)
   for j,n in enumerate(names):
    lo,hi=np.rad2deg(specs[n]['real_limits_rad'])
    if values[:,j].min()<lo-1e-5 or values[:,j].max()>hi+1e-5:raise ValueError('Hardware limit: '+n)
   peak=float(abs(np.gradient(values,ts,axis=0,edge_order=2)).max());speeds[g]=max(speeds.get(g,0),peak)
   if g in meta.get('speed_caps',{}) and peak>meta['speed_caps'][g]+1e-5:raise ValueError('Speed cap: '+g)
   cols+=names;arrays.append(np.column_stack([np.interp(t,ts,values[:,j]) for j in range(len(names))]))
  for key in ['eyou/q','eyou/t','hand/q','hand/t','arms/joint_position','arms/t_arm_js']:
   if len(f[key]):raise ValueError('Body/legacy hand/measured feedback must be empty')
 path=model_trajectory(m,base,poses)
 for j in range(m.njnt):
  if m.jnt_limited[j]:
   low,high=m.jnt_range[j];v=path[:,m.jnt_qposadr[j]]
   if v.min()<low-1e-5 or v.max()>high+1e-5:raise ValueError('Model limit: '+m.joint(j).name)
 np.savetxt(directory/'trajectory_deg.csv',np.column_stack([t,*arrays]),delimiter=',',header='t_s,'+','.join(cols),comments='',fmt='%.10f')
 report=dict(status='passed',input_sha256=inputs(directory),frames=len(t),duration_s=float(t[-1]),peak_speed_deg_s=speeds,limits_endpoints_units_and_sync_checked=True,body_commands_empty=True,feedback_empty=True)
 write(directory/'kinematic_validation.json',report);return report

def check(directory,stage='trajectory',seconds=0):
 from motion_collision import finalize_package,check_package
 from offline_backend import validate
 directory=Path(directory)
 if stage!='trajectory':return check_package(directory,stage=stage,at_seconds=seconds)
 kinematics(directory);refresh_manifest(directory)
 try:report=finalize_package(directory)
 finally:refresh_manifest(directory)
 write(directory/'backend_validation.json',validate(directory));refresh_manifest(directory)
 return {k:report[k] for k in ['status','certified_fraction','elapsed_s','checked_poses'] if k in report}

def render(directory,fps=24,video=True):
 import os
 os.environ.setdefault('MUJOCO_GL','egl')
 # Set the backend before importing MuJoCo; the CLI bootstrap does this too.
 import mujoco,imageio_ffmpeg,subprocess
 from PIL import Image,ImageDraw,ImageFont
 from preview_motion_package import load_package
 from motion_collision import model_trajectory
 directory=Path(directory);m,d,base,t,poses,meta=load_package(directory,check_collision_provenance=False)
 values=model_trajectory(m,base,poses);opt=mujoco.MjvOption();opt.geomgroup[3]=0
 cams=[]
 for az in [165.,75.]:
  c=mujoco.MjvCamera();c.azimuth=az;c.elevation=-8.;c.distance=1.55;c.lookat[:]=[.08,0,1.13];cams.append(c)
 hc=[]
 for side,az in [('left',130.),('right',200.)]:
  c=mujoco.MjvCamera();c.azimuth=az;c.elevation=-10.;c.distance=.45;hc.append((side,c))
 font=ImageFont.truetype(str(SKILL/'assets/fonts/NotoSansCJK-Regular.ttc'),18)
 with mujoco.Renderer(m,height=480,width=480) as renderer,mujoco.Renderer(m,height=240,width=320) as hr:
  def frame(seconds):
   d.qpos[:]=[np.interp(seconds,t,values[:,j]) for j in range(m.nq)];mujoco.mj_forward(m,d)
   out=Image.new('RGB',(1280,512),(22,27,34))
   for j,c in enumerate(cams):renderer.update_scene(d,camera=c,scene_option=opt);out.paste(Image.fromarray(renderer.render()),(j*480,32))
   for j,(side,c) in enumerate(hc):
    c.lookat[:]=d.xpos[m.body(side.title()+'Palm').id]+[.045,0,-.025];hr.update_scene(d,camera=c,scene_option=opt);out.paste(Image.fromarray(hr.render()),(960,32+j*240))
   ImageDraw.Draw(out).text((10,4),f'{meta["name"]} | {seconds:.2f}/{t[-1]:.2f}s | 正面 / 侧面 / 双手细节',font=font,fill='white');return out
  times=np.asarray(meta.get('preview_times_s',np.array([0,.17,.37,.58,.81,1])*t[-1]),float)
  if len(times)!=6 or not np.isfinite(times).all() or times.min()<0 or times.max()>t[-1]:times=np.array([0,.17,.37,.58,.81,1])*t[-1]
  sheet=Image.new('RGB',(2560,1536))
  for j,ts in enumerate(times):sheet.paste(frame(ts),((j%2)*1280,(j//2)*512))
  sheet.save(directory/'keyframes.jpg',quality=90)
  framecount=0
  if video:
   process=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x512','-r',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(directory/'preview.mp4')],stdin=subprocess.PIPE)
   try:
    for seconds in np.r_[np.arange(0,t[-1],1/fps),t[-1]]:process.stdin.write(frame(seconds).tobytes());framecount+=1
   finally:process.stdin.close()
   if process.wait():raise RuntimeError('Video encoder failed')
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 table=np.loadtxt(directory/'trajectory_deg.csv',delimiter=',',skiprows=1);names=(directory/'trajectory_deg.csv').read_text().splitlines()[0].split(',')[1:]
 fig,ax=plt.subplots(figsize=(12,5))
 for j,n in enumerate(names):ax.plot(table[:,0],table[:,j+1],label=n,linewidth=.8)
 ax.set(xlabel='seconds',ylabel='calibrated degrees');ax.grid(alpha=.25);ax.legend(fontsize=6,ncol=4);fig.tight_layout();fig.savefig(directory/'trajectory.png',dpi=120);plt.close(fig)
 write(directory/'render_validation.json',dict(full_duration_video=video,fps=fps,frames=framecount,duration_s=float(t[-1]),input_sha256=inputs(directory),views=['front','side','left_hand','right_hand']))
 refresh_manifest(directory)

def review(directory,notes,reviewer):
 directory=Path(directory)
 for name in ['keyframes.jpg','preview.mp4']:
  if not (directory/name).is_file():raise ValueError('Render before reviewing')
 write(directory/'visual_review.json',dict(status='passed',reviewer=reviewer,method='Explicit recorded visual review; this command does not inspect images automatically',notes=notes,input_sha256=inputs(directory)|{n:digest(directory/n) for n in ['keyframes.jpg','preview.mp4']}));refresh_manifest(directory)

def verify_export(directory):
 import cv2
 from motion_collision import validate_package_report
 from offline_backend import SOURCE,HELPER
 directory=Path(directory);manifest=read(directory/'manifest.json')
 if manifest.get('draft') or not manifest.get('collision_check_required'):raise ValueError('Complete trajectory check required')
 for n,h in manifest['files'].items():
  if digest(directory/n)!=h:raise ValueError('Package file changed: '+n)
 validate_package_report(directory,manifest)
 for file in ['kinematic_validation.json','backend_validation.json','visual_review.json']:
  report=read(directory/file)
  if report.get('status')!='passed' or not set(inputs(directory))<=set(report.get('input_sha256',{})):raise ValueError('Missing current evidence: '+file)
  for n,h in report['input_sha256'].items():
   if digest(directory/n)!=h:raise ValueError('Stale evidence: '+file+' '+n)
 backend=read(directory/'backend_validation.json')
 if backend['backend_sha256']!=digest(SOURCE) or backend['helper_sha256']!=digest(HELPER):raise ValueError('Backend validation stale')
 rv=read(directory/'render_validation.json')
 if not rv['full_duration_video'] or rv['input_sha256']!=inputs(directory):raise ValueError('Full current video required')
 video=cv2.VideoCapture(str(directory/'preview.mp4'));count=video.get(cv2.CAP_PROP_FRAME_COUNT);fps=video.get(cv2.CAP_PROP_FPS);video.set(cv2.CAP_PROP_POS_FRAMES,max(0,count-1));ok,_=video.read();video.release()
 if not ok or fps<=0 or abs((count-1)/fps-read(directory/'motion.json')['duration_s'])>1/fps+.01:raise ValueError('Incomplete video')
 return True

def publish(directory,out=None):
 directory=Path(directory);verify_export(directory)
 out=Path(out) if out else WORK/'packages'/directory.name
 if out.name!=directory.name:raise ValueError('Published package basename must stay unchanged')
 if out!=directory:
  if out.exists() or out.with_suffix('.zip').exists():raise FileExistsError(out)
  out.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(directory,out)
 archive=out.with_suffix('.zip')
 with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(out.iterdir()):
   if p.is_file():z.write(p,Path(out.name)/p.name)
 return dict(package=str(out),archive=str(archive),zip_sha256=digest(archive))

def doctor():
 import importlib.metadata,xml.etree.ElementTree as ET,mujoco
 from calibrated_hand_pose import CalibratedHandPose
 from collision_proxy import geometry_hashes
 from model_snapshot import verify as verify_snapshot
 required=['mujoco','numpy','scipy','h5py','Pillow','PyYAML','matplotlib','opencv-python','imageio-ffmpeg']
 versions={n:importlib.metadata.version(n) for n in required}
 for f in ROBOT.rglob('*'):
  if f.is_symlink() and not f.resolve().is_relative_to(SKILL):raise ValueError('External symlink: '+str(f))
 for path in [ROBOT/'generated/humanoid_shadow.xml',ROBOT/'generated/collision_proxy_doc50_20261021/model.xml']:
  for node in ET.parse(path).iter():
   if node.tag in ['mesh','texture','hfield'] and 'file' in node.attrib:
    asset=(path.parent/node.attrib['file']).resolve()
    if not asset.is_relative_to(SKILL) or not asset.is_file():raise ValueError('Missing/external model asset: '+str(asset))
 model=mujoco.MjModel.from_xml_path(str(ROBOT/'generated/humanoid_shadow.xml'));CalibratedHandPose();hand_baseline_deg();geometry_hashes(read(ROBOT/'configs/design_collision_rules.json'));verify_snapshot()
 return dict(status='passed',skill_root=str(SKILL),model_joints=model.njnt,dependencies=versions,external_project_dependencies=False,hardware_access=False)

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
 sub.add_parser('doctor')
 x=sub.add_parser('create');x.add_argument('design',type=Path);x.add_argument('--out',type=Path)
 x=sub.add_parser('check');x.add_argument('package',type=Path);x.add_argument('--stage',choices=['quick','candidate','trajectory'],default='trajectory');x.add_argument('--time',type=float,default=0)
 x=sub.add_parser('render');x.add_argument('package',type=Path);x.add_argument('--fps',type=int,default=24);x.add_argument('--keyframes-only',action='store_true')
 x=sub.add_parser('review');x.add_argument('package',type=Path);x.add_argument('--notes',required=True);x.add_argument('--reviewer',required=True)
 x=sub.add_parser('publish');x.add_argument('package',type=Path);x.add_argument('--out',type=Path)
 x=sub.add_parser('preview');x.add_argument('package',type=Path);x.add_argument('--then',type=Path,nargs='*',default=[])
 a=p.parse_args();result=None
 if a.cmd=='doctor':result=doctor()
 elif a.cmd=='create':create(a.design,a.out)
 elif a.cmd=='check':result=check(a.package,a.stage,a.time)
 elif a.cmd=='render':
  if not 1<=a.fps<=60:raise ValueError('fps must be 1..60')
  render(a.package,a.fps,not a.keyframes_only)
 elif a.cmd=='review':review(a.package,a.notes,a.reviewer)
 elif a.cmd=='publish':result=publish(a.package,a.out)
 elif a.cmd=='preview':
  import subprocess
  raise SystemExit(subprocess.call([sys.executable,str(ROBOT/'scripts/preview_motion_package.py'),str(a.package),'--then',*map(str,a.then)]))
 if result is not None:print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
