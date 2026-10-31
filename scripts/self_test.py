"""Offline behavioral checks in an isolated work directory; no hardware access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,copy,json,sys,tempfile,shutil
from pathlib import Path
import numpy as np,h5py
from common import SKILL,ROBOT,WORK,read,write,reference_fingerprint,digest
import motion,hand_reference,common
from offline_backend import validate as backend
from motion_baseline import hand_baseline_deg
from preview_motion_package import load_package

def must_fail(fn,kind=Exception):
 try:fn()
 except kind:return
 raise AssertionError('Expected a rejection')

def run(full=False):
 forbidden=[Path(x).resolve() for x in os.environ.get('SKILL_FORBID_READ_ROOT','').split(os.pathsep) if x]
 def audit(event,args):
  if event in ['socket.connect','socket.getaddrinfo']:raise AssertionError('Self-test attempted network access')
  if forbidden and event=='open' and isinstance(args[0],(str,bytes)):
   path=Path(os.fsdecode(args[0])).resolve()
   if any(path==root or path.is_relative_to(root) for root in forbidden):raise AssertionError('Read original skill/project: '+str(path))
 sys.addaudithook(audit)
 WORK.mkdir(exist_ok=True);saved=hand_reference.DEFAULT.read_bytes();checks=[]
 with tempfile.TemporaryDirectory(prefix='self_test_',dir=WORK) as folder:
  tmp=Path(folder);motion.WORK=tmp;hand_reference.WORK=tmp;common.WORK=tmp
  try:
   motion.doctor();checks.append('model_assets_calibration_and_dependencies')
   for n in read(SKILL/'assets/bundle.json')['reference_examples']:
    load_package(ROBOT/'generated/motion_packages'/n)
   checks.append('six_reference_packages_current_proofs_and_loader')
   specs={}
   for name,tracks in [('neck',{'neck':{'mode':'rest_relative','keyframes':[{'t':0,'q':{}},{'t':1,'q':{'neck_joint1':1}},{'t':2,'q':{}}]}}),('hands',{'hand_right':{'mode':'rest_relative','keyframes':[{'t':0,'q':{}},{'t':1,'q':{'index_mcp':.5}},{'t':2,'q':{}}]}})]:
    spec=dict(package='fb_self_test_'+name+'_v1',name='离线测试夹具 '+name,duration_s=2.,tracks=tracks)
    file=tmp/(name+'.json');write(file,spec);d=motion.create(file);backend(d);specs[name]=d
   # A real combined draft is used for both retargeting modes.
   spec=read(SKILL/'assets/design_small_conversation.json');spec['package']='fb_self_test_combined_v1';write(tmp/'combined.json',spec)
   combined=motion.create(tmp/'combined.json');backend(combined);checks.append('pure_neck_pure_hand_and_combination_backend_forward_reverse')
   cal=hand_reference.CalibratedHandPose();profile=read(hand_reference.DEFAULT);rows=[]
   for i in range(4):
    hands={}
    for side in ['left','right']:
     model=cal.models[side+'_hand'];q=[profile['hands'][side][n]+(.1 if side=='right' and n=='index_mcp' else 0) for n in model.names]
     hands[side]=dict(serial=cal.settings['hands'][side]['serial'],names=model.names,motor_ids=model.ids,joint_deg=q)
    rows.append(dict(timestamp_s=10+i*.1,sequence='test_fixture_'+str(i),hands=hands))
   samples=dict(source='measured_feedback',test_fixture=True,calibration_files_sha256=reference_fingerprint(),samples=rows)
   write(tmp/'samples.json',samples)
   for field,value in [('source','target_commands'),('calibration_files_sha256',{})]:
    bad=copy.deepcopy(samples);bad[field]=value;write(tmp/'bad.json',bad)
    must_fail(lambda:hand_reference.prepare(tmp/'bad.json','must_reject'))
   bad=copy.deepcopy(samples);bad['samples'][1]['sequence']=bad['samples'][0]['sequence'];write(tmp/'bad.json',bad);must_fail(lambda:hand_reference.prepare(tmp/'bad.json','must_reject'))
   checks.append('reject_target_stream_stale_calibration_and_duplicate_samples')
   ref=hand_reference.prepare(tmp/'samples.json','test_reference')
   must_fail(lambda:hand_reference.adopt(ref),FileNotFoundError)
   hand_reference.validate_reference(ref);hand_reference.adopt(ref);hand_baseline_deg();checks.append('stable_measurement_reference_validation_and_adoption')
   # Separate sources allow separate semantic policies.
   absolute=combined.with_name('fb_self_test_absolute_v1');shutil.copytree(combined,absolute)
   selection=[dict(source=str(combined),package='fb_self_test_relative_v2'),dict(source=str(absolute),package='fb_self_test_absolute_v2')]
   write(tmp/'selection.json',selection);write(tmp/'modes.json',{combined.name:{'right':'rest_relative'},absolute.name:{'right':'absolute'}})
   result=hand_reference.rebase(tmp/'selection.json',tmp/'modes.json')
   assert all(x['status'].startswith('kinematics_passed') for x in result),result
   for row,mode in zip(result,['rest_relative','absolute']):
    with h5py.File(row['directory']+'/fullbody.hdf5') as new,h5py.File(combined/'fullbody.hdf5') as old:
     for key in ['arms/cmd_left','arms/cmd_right','arms/t_arm_cmd_l','arms/t_arm_cmd_r','neck/q','neck/t','hand_right/t','hand_left/q']:np.testing.assert_array_equal(new[key][:],old[key][:])
     names=read(combined/'motion.json')['hand_joint_names']['right'];j=names.index('index_mcp');delta=new['hand_right/q'][:,j]-old['hand_right/q'][:,j]
     np.testing.assert_allclose(delta[[0,-1]],.1,atol=1e-10)
     if mode=='rest_relative':np.testing.assert_allclose(delta,.1,atol=1e-10)
     else:np.testing.assert_allclose(delta[len(delta)//2],0.,atol=1e-10)
    must_fail(lambda:motion.publish(Path(row['directory'])))
   checks.append('relative_and_semantic_absolute_retarget_without_nonhand_or_time_changes')
   checks.append('uncertified_export_rejected')
   # A calibration import invalidates the adopted reference and is reversible.
   plan=hand_reference.import_calibration(SKILL/'assets/calibration_candidates/20261030')
   assert plan['status']=='dry_run'
   files={p:(ROBOT/p).read_bytes() for p in plan['files']}
   try:
    hand_reference.import_calibration(SKILL/'assets/calibration_candidates/20261030',True)
    must_fail(hand_baseline_deg)
   finally:
    for p,data in files.items():(ROBOT/p).write_bytes(data)
   checks.append('changed_calibration_blocks_old_reference_and_import_rolls_back')
   if full:
    motion.check(Path(result[0]['directory']));motion.render(Path(result[0]['directory']),fps=12)
    must_fail(lambda:motion.publish(Path(result[0]['directory'])))
    checks.append('full_path_render_and_missing_visual_review_export_guard')
   # Check source assets did not silently change as a side effect of retargeting.
   assert all((ROBOT/p).read_bytes()==b for p,b in files.items())
  finally:hand_reference.DEFAULT.write_bytes(saved)
 return dict(status='passed',full=full,checks=checks,original_project_required=False,network_access=False,robot_commands_sent=False,synthetic_samples_used_only_inside_isolated_tests=True)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--full',action='store_true');p.add_argument('--report',type=Path);a=p.parse_args();result=run(a.full)
 if a.report:write(a.report,result)
 print(json.dumps(result,ensure_ascii=False,indent=2))
