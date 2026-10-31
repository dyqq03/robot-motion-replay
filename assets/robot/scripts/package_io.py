"""Portable canonical command exporter, adapted from the existing exporter."""
from pathlib import Path
import json,shutil
import numpy as np,h5py,mujoco
from doc50_kinematics import Designer,ARM_NAMES,NECK_NAMES,SIDES,ROOT
from motion_baseline import require_endpoints,require_hand_endpoints,read_baseline
from collision_checker import digest
from motion_collision import require_new_package
RULES=ROOT/'configs/design_collision_rules.json'
def write_json(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def validate(designer,t,q,meta):
 if not np.isfinite(q).all() or not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Invalid finite timeline')
 np.testing.assert_allclose(q[[0,-1]],np.tile(designer.rest,(2,1)),rtol=0,atol=1e-10)
 require_endpoints(q[:,:12],ARM_NAMES)
 require_endpoints(q[:,12:14],NECK_NAMES,group='neck')
 velocity=np.gradient(q,t,axis=0,edge_order=2)
 acceleration=np.gradient(velocity,t,axis=0,edge_order=2)
 limits={}
 for mapping in (designer.mapping,designer.neck_mapping,designer.cal.mapping):
  for row in mapping['joints']:limits[row['source_joint']]=np.rad2deg(row['real_limits_rad'])
 names=ARM_NAMES+NECK_NAMES+[s+'/'+n for s in SIDES for n in designer.hand_names[s]]
 margins=[]
 for i,n in enumerate(names):
  lo,hi=limits[n];margin=float(min(q[:,i].min()-lo,hi-q[:,i].max()));margins.append(margin)
  if margin< -1e-5:raise ValueError(f'{n} outside hardware limits by {margin} deg')
 groups={'arms':list(range(12)),'neck':[12,13],'hand_left':list(range(14,30)),'hand_right':list(range(30,46))}
 for group,cap in meta['speed_caps'].items():
  if abs(velocity[:,groups[group]]).max()>cap+1e-5:raise ValueError('Speed cap: '+group)
 # Uncommanded values are assumptions for collision/visualization, never streams.
 if not meta['arms']:np.testing.assert_allclose(q[:,:12],np.tile(designer.rest[:12],(len(t),1)),atol=1e-10)
 if not meta['neck']:np.testing.assert_allclose(q[:,12:14],np.tile(designer.rest[12:14],(len(t),1)),atol=1e-10)
 for side in SIDES:
  off=14+(0 if side=='left' else 16)
  if side not in meta['hands']:
   np.testing.assert_allclose(q[:,off:off+16],np.tile(designer.rest[off:off+16],(len(t),1)),atol=1e-10)
  elif designer.hand_reference is not None:require_hand_endpoints(q[:,off:off+16],designer.hand_names[side],side=side)
 modelq=designer.model_path(q)
 for j in range(designer.model.njnt):
  if designer.model.jnt_limited[j]:
   low,high=designer.model.jnt_range[j];col=designer.model.jnt_qposadr[j]
   if modelq[:,col].min()<low-1e-5 or modelq[:,col].max()>high+1e-5:raise ValueError('Model limits: '+designer.model.joint(j).name)
 return dict(status='passed',frames=len(t),duration_s=float(t[-1]),units_valid=True,names_valid=True,synchronized=True,start_end_baselines_valid=True,hardware_and_model_limits_valid=True,minimum_hardware_limit_margin_deg=min(margins),peak_speed_deg_s={g:float(abs(velocity[:,v]).max()) for g,v in groups.items()},peak_acceleration_deg_s2={g:float(abs(acceleration[:,v]).max()) for g,v in groups.items()},speed_caps_deg_s=meta['speed_caps'],body_commands_empty=True,no_measured_feedback_fabricated=True,hardware_execution_validated=False)

def write_draft(directory,designer,t,q,meta):
 require_new_package(directory);directory.mkdir(parents=True)
 checks=validate(designer,t,q,meta)
 arm_active=bool(meta['arms']);neck_active=meta['neck'];hand_names={s:designer.hand_names[s] for s in meta['hands']}
 with h5py.File(directory/'fullbody.hdf5','w') as f:
  f.attrs.update(source_bag=directory.name,arm_mode='ker',duration_sec=float(t[-1]),synthetic=True,sample_hz=(len(t)-1)/t[-1])
  a=f.create_group('arms');a.attrs.update(mode='ker',joint_names=ARM_NAMES if arm_active else [],units='rad',cmd_source='synthetic_calibrated_document_gesture')
  for side,off,suffix in (('left',0,'l'),('right',6,'r')):
   a.create_dataset('cmd_'+side,data=np.deg2rad(q[:,off:off+6]) if arm_active else np.empty((0,6)))
   a.create_dataset('t_arm_cmd_'+suffix,data=t if arm_active else np.empty(0))
   a.create_dataset('gaps_arm_cmd_'+suffix,data=np.empty((0,2)))
  a.create_dataset('joint_position',data=np.empty((0,12)));a.create_dataset('t_arm_js',data=np.empty(0));a.create_dataset('gaps_arm_js',data=np.empty((0,2)))
  for group,width in (('neck',2),('eyou',4),('hand',1),('hand_left',16),('hand_right',16)):
   g=f.create_group(group);active=neck_active if group=='neck' else group.removeprefix('hand_') in hand_names
   off=12 if group=='neck' else 14 if group=='hand_left' else 30
   g.create_dataset('q',data=(np.deg2rad(q[:,12:14]) if group=='neck' else q[:,off:off+width]) if active else np.empty((0,width)))
   g.create_dataset('t',data=t if active else np.empty(0));g.create_dataset('gaps',data=np.empty((0,2)));g.attrs['units']='rad' if group=='neck' else 'deg'
   if active:
    g.attrs['joint_names']=np.array(NECK_NAMES if group=='neck' else hand_names[group.removeprefix('hand_')],dtype=h5py.string_dtype('utf-8'))
    if group!='neck':g.attrs['calibration_revision']=designer.cal.revision
 chains={}
 if arm_active:chains['arms']=dict(mode='ker',frames_cmd_l=len(t),frames_cmd_r=len(t),frames_js=0,units='rad')
 else:chains['arms']=dict(mode='ker',frames_cmd_l=0,frames_cmd_r=0,frames_js=0,units='rad')
 for g in ('neck','eyou','hand','hand_left','hand_right'):
  active=neck_active if g=='neck' else g.removeprefix('hand_') in hand_names
  chains[g]=dict(frames=len(t) if active else 0,units='rad' if g=='neck' else 'deg')
  if g.startswith('hand_') and active:chains[g]['joint_names']=hand_names[g[5:]]
 write_json(directory/'fullbody_meta.json',dict(source_bag=directory.name,arm_mode='ker',duration_sec=float(t[-1]),synthetic=True,missing_joints=[],name_reorders=0,dropped_jump_frames=0,gaps={},chains=chains,hand_calibration_revision=designer.cal.revision))
 motion=meta|dict(schema_version=1,duration_s=float(t[-1]),sample_hz=float((len(t)-1)/t[-1]),synthetic=True,joint_names=ARM_NAMES if arm_active else [],neck_joint_names=NECK_NAMES if neck_active else [],hand_joint_names=hand_names,preview_arm_baseline_rad=dict(zip(ARM_NAMES,np.deg2rad(designer.arm_rest).tolist())),preview_neck_baseline_rad=dict(zip(NECK_NAMES,np.deg2rad(designer.neck_rest).tolist())),baseline_id=read_baseline()['id'],hand_calibration_revision=designer.cal.revision,preview_cycles=0,preview_label=f'LOCAL {meta["index"]:02d} {meta["slug"]}',loop_endpoints_match=True,units=dict(arms='rad',neck='rad',hand_left='deg',hand_right='deg',trajectory_csv='deg'),interpolation='C2 quintic envelopes and head splines / bounded cubic arm B-splines; server and collision path: piecewise linear between dense commands',uncommanded_pose_assumptions=dict(arms='current configured baseline snapshot in preview_arm_baseline_rad',neck=f'J1 {designer.neck_rest[0]:g} deg, J2 {designer.neck_rest[1]:g} deg',hands='declared hand reference snapshot',waist_legs='model qpos0'),bilateral_arm_format_holds_inactive_side=arm_active and len(meta['arms'])==1,hardware_execution_validated=False,generator='portable robot-motion-replay')
 if designer.hand_reference is not None:
  motion['hand_baseline_id']=json.loads((ROOT/'configs/hand_motion_defaults.json').read_text())['id']
  motion['hand_baseline_pose_deg']={s:dict(zip(designer.hand_names[s],designer.hand_rest[s].tolist())) for s in SIDES}
  motion['uncommanded_pose_assumptions']['hands']='measured hand reference snapshot in hand_baseline_pose_deg'
 write_json(directory/'motion.json',motion);write_json(directory/'kinematic_validation.json',checks)
 shutil.copyfile(RULES,directory/'collision_rules.json')
 columns=[];values=[t]
 if arm_active:columns+=ARM_NAMES;values.append(q[:,:12])
 if neck_active:columns+=NECK_NAMES;values.append(q[:,12:14])
 for side in meta['hands']:
  off=14+(0 if side=='left' else 16);columns+=[side+'/'+n for n in hand_names[side]];values.append(q[:,off:off+16])
 np.savetxt(directory/'trajectory_deg.csv',np.column_stack(values),delimiter=',',header='t_s,'+','.join(columns),comments='',fmt='%.10f')
 write_json(directory/'source.json',dict(source=meta.get('source','user_requested_design'),excerpt=meta['source_excerpt'],interpretation='空手示意；身体/腰腿指令为空'))
 (directory/'README.md').write_text(f'''# {meta['index']:02d} {meta['name']}

来源：{('动作文档第'+str(meta['source_page'])+'页，') if meta.get('source_page') is not None else ''}{meta['source_excerpt']}。时长 {t[-1]:.2f} 秒。
手臂：{meta['arms']}；颈部：{neck_active}；手指：{meta['hands']}。
手臂/颈部命令单位为弧度，手指为标定关节角度“度”；实测反馈为空，不需要.db3。
首尾回到约定基准，有手指指令时首尾采用包内声明的实测参考手型。身体/腰腿命令为空。
单侧手臂动作沿用双臂格式，另一侧明确保持实测基准。其他未参与链不发送指令，碰撞检查采用motion.json声明的假设姿态。

在技能根目录预览：`python scripts/motion.py preview {directory.relative_to(ROOT.parents[1])}`
视频 preview.mp4；正侧面关键帧 keyframes.jpg；角度轨迹 trajectory_deg.csv / trajectory.png。
完整路径检查见 collision_validation.json，装配规则见 collision_rules.json。
仅本地运动学预览与模型自碰撞检查；无真机、动力学、运行时起播过渡或外部障碍物验证。
''',encoding='utf-8')
 manifest=dict(schema_version=1,draft=True,collision_check_required=False,collision_rules_file='collision_rules.json',model_sha256=digest(ROOT/'generated/humanoid_shadow.xml'),arm_mapping_sha256=digest(ROOT/'configs/arm_mapping.json'),neck_mapping_sha256=digest(ROOT/'configs/neck_mapping.json'),hand_mapping_sha256=digest(ROOT/'configs/hand_mapping.json'),hand_hardware_sha256=digest(ROOT/'configs/hand_hardware.json'),hand_calibration_revision=designer.cal.revision,files={})
 write_json(directory/'manifest.json',manifest);rehash(directory)
 return checks

def rehash(directory):
 p=directory/'manifest.json';m=json.loads(p.read_text());m['files']={f.name:digest(f) for f in sorted(directory.iterdir()) if f.is_file() and f.name!='manifest.json'};write_json(p,m)
