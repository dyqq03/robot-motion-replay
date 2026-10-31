"""Rebind copied reference metadata and run actual checks within this bundle."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,h5py
from pathlib import Path
from common import ROBOT,SKILL,read,write,digest,refresh_manifest
from calibrated_hand_pose import CalibratedHandPose
from motion import check,render

def main():
 p=argparse.ArgumentParser();p.add_argument('--start',type=int,default=0);p.add_argument('--end',type=int,default=6);p.add_argument('--no-render',action='store_true');a=p.parse_args()
 bundle=read(SKILL/'assets/bundle.json');results=[];cal=CalibratedHandPose()
 for name in bundle['reference_examples'][a.start:a.end]:
  d=ROBOT/'generated/motion_packages'/name;m=read(d/'motion.json');m.update(hand_calibration_revision=cal.revision,bundle_reference_copy=True)
  write(d/'motion.json',m);fm=read(d/'fullbody_meta.json');fm['hand_calibration_revision']=cal.revision;write(d/'fullbody_meta.json',fm)
  with h5py.File(d/'fullbody.hdf5','r+') as f:
   for side in ['left','right']:
    if len(f['hand_'+side+'/q']):f['hand_'+side].attrs['calibration_revision']=cal.revision
  manifest=read(d/'manifest.json')
  for filename,key in [('arm_mapping.json','arm_mapping_sha256'),('neck_mapping.json','neck_mapping_sha256'),('hand_mapping.json','hand_mapping_sha256'),('hand_hardware.json','hand_hardware_sha256')]:manifest[key]=digest(ROBOT/'configs'/filename)
  manifest.update(draft=True,collision_check_required=False,hand_calibration_revision=cal.revision);write(d/'manifest.json',manifest)
  write(d/'collision_rules.json',read(ROBOT/'configs/design_collision_rules.json'))
  for filename in ['visual_review.json','collision_validation.json','backend_validation.json']:(d/filename).unlink(missing_ok=True)
  refresh_manifest(d)
  result=check(d);print(name,result,flush=True)
  if not a.no_render:render(d,fps=24)
  results.append(dict(package=name,**result))
 write(SKILL/'work'/f'example_validation_{a.start}_{a.end}.json',results)
if __name__=='__main__':main()
