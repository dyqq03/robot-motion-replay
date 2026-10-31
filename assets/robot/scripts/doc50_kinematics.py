"""Offline pose design helpers for the 2026-10-21 document batch."""
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from calibrated_hand_pose import CalibratedHandPose
from motion_baseline import arm_baseline_deg,neck_baseline_deg
from view_shadow_model import arm_positions,apply_pose
ROOT=Path(__file__).resolve().parents[1]
SIDES=('left','right')
ARM_NAMES=[f'openarm_{s}_joint{j}' for s in SIDES for j in (1,2,3,4,5,7)]
NECK_NAMES=['neck_joint1','neck_joint2']

class Designer:
 def __init__(self,hand_reference=None):
  self.model=mujoco.MjModel.from_xml_path(str(ROOT/'generated/humanoid_shadow.xml'))
  self.data=mujoco.MjData(self.model)
  self.cal=CalibratedHandPose()
  self.mapping=json.loads((ROOT/'configs/arm_mapping.json').read_text())
  self.neck_mapping=json.loads((ROOT/'configs/neck_mapping.json').read_text())
  self.arm_rest=np.array([arm_baseline_deg()[n] for n in ARM_NAMES])
  self.neck_rest=np.array([neck_baseline_deg()[n] for n in NECK_NAMES])
  self.hand_names={s:list(self.cal.models[s+'_hand'].names) for s in SIDES}
  self.hand_reference=hand_reference
  self.hand_rest={s:np.array([hand_reference[s+'/'+n] for n in self.hand_names[s]]) if hand_reference is not None else np.zeros(16) for s in SIDES}
  self.rest=np.r_[self.arm_rest,self.neck_rest,self.hand_rest['left'],self.hand_rest['right']]
  self.base=self.cal.zero_model_pose()|arm_positions(dict(zip(ARM_NAMES,np.deg2rad(self.arm_rest))),self.mapping)|arm_positions(dict(zip(NECK_NAMES,np.deg2rad(self.neck_rest))),self.neck_mapping)
  if hand_reference is not None:self.base.update(self.cal.from_joint_degrees(hand_reference))
  self.pose_log=[]
 def fk(self,side,angles):
  off=0 if side=='left' else 6
  apply_pose(self.model,self.data,self.base|arm_positions(dict(zip(ARM_NAMES[off:off+6],np.deg2rad(angles))),self.mapping))
  body=self.model.body(side.title()+'Palm').id
  return self.data.xpos[body].copy(),self.data.xmat[body].reshape(3,3).copy()
 def ik(self,side,position,normal=None,seed=None,free_wrist=False):
  off=0 if side=='left' else 6
  names=ARM_NAMES[off:off+6];rest=self.arm_rest[off:off+6]
  specs={x['source_joint']:x for x in self.mapping['joints']}
  lo,hi=np.rad2deg(np.array([specs[n]['real_limits_rad'] for n in names]).T)
  lo+=1.;hi-=1.
  if side=='right':lo[1]=max(lo[1],15.)
  else:hi[1]=min(hi[1],-15.)
  hi[3]=min(hi[3],125.)
  lo[4]=max(lo[4],-72.);hi[4]=min(hi[4],72.)
  lo[5]=max(lo[5],rest[5]-40.);hi[5]=min(hi[5],rest[5]+45.)
  if side=='right':hi[5]=min(hi[5],43.)
  count=6 if free_wrist else 5
  # Avoid elbow lock and preserve outward shoulder space.
  lo[3]=max(lo[3],15.)
  best=None
  seed=rest if seed is None else np.array(seed)
  starts=[seed,rest+np.array([-15 if side=='left' else 15,-10 if side=='left' else 10,0,45,0,0])]
  if position[2]>1.25:
   starts.append(np.array([-40,60,75,100,15,55]) * (np.array([-1,-1,-1,1,-1,-1]) if side=='left' else 1))
  for start in starts:
   def unpack(x):return np.r_[x,rest[5]] if count==5 else x
   def residual(x):
    q=unpack(x);p,r=self.fk(side,q)
    terms=[(p-position)*10.]
    if normal is not None:terms.append((r[:,1]-normal)*.45)
    terms.append((q[:count]-seed[:count])*.0005)
    return np.concatenate(terms)
   res=least_squares(residual,np.clip(start[:count],lo[:count]+1e-5,hi[:count]-1e-5),bounds=(lo[:count],hi[:count]),max_nfev=130,ftol=1e-7)
   q=unpack(res.x);p,r=self.fk(side,q)
   score=np.linalg.norm(p-position)+(0 if normal is None else .04*np.linalg.norm(r[:,1]-normal))
   if best is None or score<best[0]:best=(score,q,p,r)
  _,q,p,r=best
  self.pose_log.append(dict(side=side,target_position_m=np.asarray(position).tolist(),actual_position_m=p.tolist(),position_error_m=float(np.linalg.norm(p-position)),angles_deg=q.tolist(),normal=r[:,1].tolist(),j7_fixed=not free_wrist))
  return q
 def model_path(self,q):
  q=np.asarray(q)
  result=np.tile(self.model.qpos0,(len(q),1))
  addresses={self.model.joint(j).name:int(self.model.jnt_qposadr[j]) for j in range(self.model.njnt)}
  for n,v in self.base.items():result[:,addresses[n]]=v
  for mapping,columns in ((self.mapping,range(12)),(self.neck_mapping,range(12,14))):
   lookup={s['source_joint']:s for s in mapping['joints']}
   names=ARM_NAMES if len(columns)==12 else NECK_NAMES
   for col,n in zip(columns,names):
    s=lookup[n];result[:,addresses[s['model_joint']]]=np.deg2rad(q[:,col])*s['sign']+s['offset_rad']
  for i,side in enumerate(SIDES):
   for j,n in enumerate(self.hand_names[side]):
    s=self.cal.specs[side+'/'+n]
    result[:,addresses[s['model_joint']]]=np.deg2rad(q[:,14+i*16+j])*s['sign']+s['offset_rad']
  return result
