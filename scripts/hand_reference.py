"""Stable measured hand references, local calibration import and versioned retargeting."""
import argparse,copy,datetime,hashlib,json,math,re,shutil,time,urllib.request
from pathlib import Path
import numpy as np,h5py,yaml
from common import SKILL,ROBOT,WORK,read,write,digest,inputs,reference_fingerprint,refresh_manifest
from calibrated_hand_pose import CalibratedHandPose
from doc50_kinematics import Designer
from collision_checker import CollisionChecker
from motion_collision import implementation_hashes,proxy_hashes

SIDES=['left','right'];DEFAULT=ROBOT/'configs/hand_motion_defaults.json';RULES=ROBOT/'configs/design_collision_rules.json'
def flat(hands):return {s+'/'+n:v for s,p in hands.items() for n,v in p.items()}
def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def stamp():return datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
def safe_id(value):
 if not re.fullmatch('[A-Za-z0-9_]+',value):raise ValueError('Reference id must contain letters, digits and underscores')
 return value

def normalize_http(payload):
 """Adapter for the bundled project's 8092 /api/ui/snapshot feedback schema."""
 cal=CalibratedHandPose();hands={};seq=[]
 for side in SIDES:
  e=payload['latest']['feedback:'+side+'_hand'];data=e['data']
  if e.get('kind')!='feedback' or e.get('valid') is not True or not 0<=e.get('age_ms',math.inf)<=500:raise ValueError('Missing/stale measured hand feedback: '+side)
  if data.get('unit')!='deg' or data.get('calibration_revision')!=cal.revision:raise ValueError('Feedback calibration/unit differs from local bundle')
  if data.get('serial')!=cal.settings['hands'][side]['serial'] or not all(data.get('valid_mask',[])) or len(data.get('valid_mask',[]))!=16:raise ValueError('Wrong hand identity or incomplete feedback')
  hands[side]=dict(names=data['name'],joint_deg=data['position'],motor_ids=data['motor_id'],serial=data['serial'])
  seq.append(str(e['monotonic_ns']))
 return dict(timestamp_s=time.time(),sequence=':'.join(seq),hands=hands)

def capture(url,seconds,hz,out):
 if not math.isfinite(seconds) or not .3<=seconds<=30 or not 1<=hz<=25:raise ValueError('Capture 0.3..30 seconds at 1..25 Hz')
 rows=[];errors=[];deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  try:
   with urllib.request.urlopen(url,timeout=3) as r:row=normalize_http(json.load(r))
   if not rows or rows[-1]['sequence']!=row['sequence']:rows.append(row)
  except Exception as exc:errors.append(str(exc))
  time.sleep(1/hz)
 result=dict(schema_version=1,source='measured_feedback',collected_at_utc=now(),calibration_files_sha256=reference_fingerprint(),samples=rows,rejected_samples=errors)
 write(out,result);return result

def prepare(samples_path,identifier,adjustments=None,max_spread=.6,max_adjustment=3.):
 data=read(samples_path);cal=CalibratedHandPose();safe_id(identifier)
 if data.get('source')!='measured_feedback':raise ValueError('Target commands/synthetic poses are not measured feedback')
 if data.get('calibration_files_sha256')!=reference_fingerprint():raise ValueError('Measurements do not bind the active calibration files')
 rows=data['samples']
 if len(rows)<3 or len({r['sequence'] for r in rows})!=len(rows):raise ValueError('At least three distinct measured samples are required')
 ts=np.array([r['timestamp_s'] for r in rows],float)
 if not np.isfinite(ts).all() or np.any(np.diff(ts)<=0) or ts[-1]-ts[0]<.2:raise ValueError('Stable measurements must cover at least 0.2 s with increasing timestamps')
 if not math.isfinite(max_spread) or max_spread<=0 or not math.isfinite(max_adjustment) or max_adjustment<0:raise ValueError('Invalid stability/adjustment bound')
 raw={};spread={};ids={}
 for side in SIDES:
  model=cal.models[side+'_hand'];samples=[]
  for row in rows:
   p=row['hands'][side]
   if p.get('errors') and any(p['errors']):raise ValueError('Motor read errors present')
   if p.get('serial')!=cal.settings['hands'][side]['serial']:raise ValueError('Measured USB side identity differs')
   if len(p['names'])!=16 or set(p['names'])!=set(model.names) or len(p['joint_deg'])!=16 or len(p['motor_ids'])!=16:raise ValueError('All 16 named joints per hand are required')
   by_id=dict(zip(p['names'],p['motor_ids']))
   if any(by_id[n]!=mid for n,mid in zip(model.names,model.ids)):raise ValueError('Joint/motor mapping differs')
   pmap=dict(zip(p['names'],p['joint_deg']))
   if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pmap.values()):raise ValueError('Non-finite measured joint angle')
   samples.append([pmap[n] for n in model.names])
  q=np.array(samples);span=np.ptp(q,axis=0)
  if span.max()>max_spread:raise ValueError('Hand is not stable: '+side+' maximum range '+str(span.max()))
  raw[side]=dict(zip(model.names,np.median(q,axis=0).tolist()));spread[side]=dict(zip(model.names,span.tolist()));ids[side]=dict(zip(model.names,model.ids))
 adopted=copy.deepcopy(raw);changes=[]
 corrections=read(adjustments) if adjustments else {}
 if set(corrections)-set(SIDES):raise ValueError('Unknown correction side')
 for side,values in corrections.items():
  if set(values)-set(adopted[side]):raise ValueError('Unknown correction joint')
  for name,value in values.items():
   if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError('Finite correction target required')
   delta=value-raw[side][name]
   if abs(delta)>max_adjustment:raise ValueError('Correction exceeds explicitly selected bound: '+side+'/'+name)
   adopted[side][name]=float(value);changes.append(dict(side=side,joint=name,measured_deg=raw[side][name],reference_deg=value,delta_deg=delta))
 out=WORK/'hand_references'/identifier
 if out.exists():raise FileExistsError(out)
 out.mkdir(parents=True);shutil.copy2(samples_path,out/'raw_samples.json')
 profile=dict(schema_version=1,id=identifier,unit='deg',coordinate_system='calibrated_hardware_joint_absolute_angles',calibration_files_sha256=reference_fingerprint(),calibration_revision=cal.revision,motor_ids_by_joint=ids,hands=adopted,raw_measured_hands=raw,measured_at_utc=data.get('collected_at_utc'),source='User accepted initial hand shape; stable measured medians',source_file='raw_samples.json',source_sha256=digest(out/'raw_samples.json'),stability_range_deg=spread,adjustment=dict(changes=changes,explicit_bound_deg=max_adjustment),changes_calibration=False,changes_startup_homing=False)
 write(out/'reference.json',profile);return out/'reference.json'

def reference_evidence(profile):
 return dict(profile_sha256=digest(profile),calibration_files_sha256=reference_fingerprint(),model_sha256=digest(ROBOT/'generated/humanoid_shadow.xml'),mapping_sha256=digest(ROBOT/'configs/hand_mapping.json'),rules_sha256=digest(RULES),proxy_hashes=proxy_hashes(RULES),implementation_sha256=implementation_hashes(),reference_tool_sha256=digest(Path(__file__)))

def validate_reference(profile):
 profile=Path(profile);p=read(profile)
 if p['calibration_files_sha256']!=reference_fingerprint():raise ValueError('Reference calibration no longer matches')
 d=Designer(hand_reference=flat(p['hands']));path=d.model_path(np.tile(d.rest,(2,1)))
 report=CollisionChecker(d.model,RULES).check_trajectory(np.array([0.,1.]),path)
 report.update(evidence=reference_evidence(profile),scope='Rest pose only; every retargeted complete trajectory still requires a new check')
 write(profile.parent/'reference_validation.json',report)
 if report['status']!='passed' or report.get('certified_fraction')!=1:raise ValueError('Reference pose requires further adjustment: '+json.dumps(report.get('findings',[])[:2]))
 return report

def adopt(profile):
 profile=Path(profile);report=read(profile.parent/'reference_validation.json')
 if report.get('status')!='passed' or report.get('certified_fraction')!=1 or report.get('evidence')!=reference_evidence(profile):raise ValueError('A current passing reference check is required')
 before=WORK/'backups'/('hand_reference_'+stamp());before.mkdir(parents=True);shutil.copy2(DEFAULT,before/DEFAULT.name)
 p=read(profile);p['source_file']='work/hand_references/'+profile.parent.name+'/raw_samples.json'
 write(DEFAULT,p)
 # No startup/controller/calibration or existing motion package is changed.
 return dict(status='adopted_local_generation_default',id=p['id'],backup=str(before),startup_changed=False,existing_packages_changed=False)

def import_calibration(source,apply=False):
 """Import a user-supplied two-hand YAML bundle locally; preserve model geometry/ROM."""
 from hand_io import HandCalibration
 source=Path(source);mapping=read(ROBOT/'configs/hand_mapping.json');settings=read(ROBOT/'configs/hand_hardware.json');updates={};hands={}
 for side in SIDES:
  files={n:(source/('orcahand_v1_'+side)/n).read_bytes() for n in ['config.yaml','calibration.yaml']};cfg=yaml.safe_load(files['config.yaml']);cal=yaml.safe_load(files['calibration.yaml'])
  if cfg.get('type')!=side:raise ValueError('Wrong hand side in source YAML')
  model=HandCalibration(cfg,cal);byname={s['source_name']:s for s in mapping['joints'] if s['side']==side}
  if set(byname)!=set(model.names):raise ValueError('Joint topology/name changes require model review')
  serial=settings['hands'][side]['serial'];port=cfg.get('port','')
  if '/by-id/' in port and serial not in port:raise ValueError('USB serial changed; confirm side identities before import')
  hashes={n:hashlib.sha256(raw).hexdigest() for n,raw in files.items()};relative='configs/hand_models/orcahand_v1_'+side
  hands[side]=dict(serial=serial,model_path=relative,source_path=relative,source_sha256=hashes,parameter_sha256=hashes)
  for i,name in enumerate(model.names):
   row=byname[name];rom=model.rom[i]
   if any(abs(math.radians(v)-q)>1e-6 for v,q in zip(rom,row['real_limits_rad'])):raise ValueError('ROM changed: update/review URDF and collision model separately')
   if cfg['neutral_position'][name]!=0:raise ValueError('Nonzero neutral requires explicit model alignment review')
   row.update(motor_id=model.ids[i],motor_direction=-1 if model.inverted[i] else 1,hardware_serial=serial,parameter_sha256=hashes,hardware_rom_deg=rom,motor_limits_rad=model.limits[i],joint_to_motor_ratio_rad_per_deg=model.ratios[i],encoder_position_at_joint_zero_rad=model.limits[i][0]+(rom[1] if model.inverted[i] else -rom[0])*model.ratios[i])
  updates.update({relative+'/'+n:raw for n,raw in files.items()})
 identity={s:{k:v[k] for k in ['serial','parameter_sha256']} for s,v in hands.items()};revision=hashlib.sha256(json.dumps(identity,sort_keys=True,separators=(',',':')).encode()).hexdigest()
 settings['calibration_revision']=revision;mapping['hardware_calibration_revision']=revision;mapping['hardware_calibration_sources']=hands
 for row in mapping['joints']:row['hardware_calibration_revision']=revision
 zero=read(ROBOT/'configs/hand_zero_readings.json');zero.update(hardware_calibration_revision=revision,hardware_calibration_source='configs/hand_models',encoder_positions_at_joint_zero_rad={s['source_joint']:s['encoder_position_at_joint_zero_rad'] for s in mapping['joints']})
 for path,data in {'configs/hand_hardware.json':settings,'configs/hand_mapping.json':mapping,'configs/hand_zero_readings.json':zero,'configs/hand_calibration.json':dict(schema_version=1,revision=revision,hands=hands,generated_at=now(),source='User supplied bundle copied locally')}.items():updates[path]=(json.dumps(data,ensure_ascii=False,indent=2)+'\n').encode()
 model_report=read(ROBOT/'generated/model_report.json');model_report['hand_calibration_revision']=revision
 model_report['joint_mapping_groups']['hand']['sha256']=hashlib.sha256(updates['configs/hand_mapping.json']).hexdigest()
 model_report['calibration_import_note']='Encoder conversion updated after ROM, topology and neutral-coordinate checks; geometry unchanged.'
 updates['generated/model_report.json']=(json.dumps(model_report,ensure_ascii=False,indent=2)+'\n').encode()
 plan=dict(status='dry_run',revision=revision,files=sorted(updates),new_reference_required=True,geometry_unchanged=True,hardware_writes=False)
 if not apply:return plan
 backup=WORK/'backups'/('calibration_'+stamp());backup.mkdir(parents=True)
 for rel in updates:
  target=backup/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROBOT/rel,target)
 try:
  for rel,raw in updates.items():(ROBOT/rel).write_bytes(raw)
  CalibratedHandPose()
  from model_snapshot import verify
  verify()
 except Exception:
  for rel in updates:shutil.copy2(backup/rel,ROBOT/rel)
  raise
 plan.update(status='imported_locally',backup=str(backup));write(WORK/'calibration_import.json',plan)
 return plan

def rebase(selection,modes,reviewed_calibration_targets=False):
 """Preserve every non-hand dataset and timestamp, emit drafts with explicit modes."""
 from motion import identifier,kinematics
 from motion_baseline import hand_baseline_deg
 ref=read(DEFAULT);hand_baseline_deg();rows=read(selection);policies=read(modes) if modes else {};results=[]
 for row in rows:
  source=Path(row['source']).resolve();name=identifier(row['package']);dest=WORK/'drafts'/name
  try:
   if not source.is_relative_to(SKILL):raise ValueError('First copy source packages inside this self-contained skill')
   if dest.exists() or dest.with_suffix('.zip').exists():raise FileExistsError(dest)
   meta=read(source/'motion.json');sides=list(meta.get('hand_joint_names',{}))
   if not sides:results.append(dict(source=source.name,status='skipped_no_hand_commands'));continue
   calibration_changed=meta.get('hand_calibration_revision')!=CalibratedHandPose().revision
   if calibration_changed and not reviewed_calibration_targets:raise ValueError('Calibration changed: review middle gesture targets and pass --calibration-targets-reviewed')
   modes_by_side=policies.get(source.name,meta.get('hand_retarget_modes',{}))
   if set(sides)-set(modes_by_side):raise ValueError('Declare rest_relative/absolute for each commanded hand; do not guess from names')
   dest.parent.mkdir(parents=True,exist_ok=True);dest.mkdir()
   for n in ['fullbody.hdf5','fullbody_meta.json','motion.json','collision_rules.json','source.json','README.md']:
    if (source/n).is_file():shutil.copy2(source/n,dest/n)
   changes=[]
   with h5py.File(dest/'fullbody.hdf5','r+') as f,h5py.File(source/'fullbody.hdf5') as orig:
    for side in sides:
     key='hand_'+side;q=f[key+'/q'][:];ts=f[key+'/t'][:];names=meta['hand_joint_names'][side]
     old=np.array([meta.get('hand_baseline_pose_deg',{}).get(side,{}).get(n,0.) for n in names]);new=np.array([ref['hands'][side][n] for n in names])
     np.testing.assert_allclose(q[[0,-1]],np.tile(old,(2,1)),rtol=0,atol=1e-8)
     mode=modes_by_side[side]
     if mode=='rest_relative':updated=q+(new-old)
     elif mode=='absolute':
      fraction=float(row.get('endpoint_blend_fraction',meta.get('hand_endpoint_blend_fraction',.22)))
      if not .05<=fraction<=.45:raise ValueError('Endpoint blend fraction must be 0.05..0.45')
      u=(ts-ts[0])/(ts[-1]-ts[0]);rise=np.clip(u/fraction,0,1);fall=np.clip((1-u)/fraction,0,1)
      smooth=lambda x:x**3*(10+x*(-15+6*x))
      envelope=smooth(rise)*smooth(fall);updated=q+(1-envelope[:,None])*(new-old)
     else:raise ValueError('Unknown retarget mode: '+str(mode))
     f[key+'/q'][:]=updated;f[key].attrs['calibration_revision']=CalibratedHandPose().revision
     changes.append(dict(side=side,mode=mode,max_endpoint_delta_deg=float(abs(new-old).max())))
    # Assert exact preservation of every dataset except the explicitly commanded hand angles.
    def verify(key,obj):
     if isinstance(obj,h5py.Dataset) and key not in ['hand_'+s+'/q' for s in sides]:np.testing.assert_array_equal(f[key][:],obj[:])
    orig.visititems(verify)
   meta.update(supersedes=source.name,hand_baseline_id=ref['id'],hand_baseline_pose_deg=ref['hands'],hand_calibration_revision=CalibratedHandPose().revision,hand_retarget_modes=modes_by_side,hand_reference_update=changes,hardware_execution_validated=False)
   write(dest/'motion.json',meta);fm=read(dest/'fullbody_meta.json');fm['hand_calibration_revision']=CalibratedHandPose().revision;write(dest/'fullbody_meta.json',fm)
   manifest=read(source/'manifest.json');manifest.update(draft=True,collision_check_required=False,model_sha256=digest(ROBOT/'generated/humanoid_shadow.xml'),hand_mapping_sha256=digest(ROBOT/'configs/hand_mapping.json'),hand_hardware_sha256=digest(ROBOT/'configs/hand_hardware.json'),hand_calibration_revision=CalibratedHandPose().revision,files={});write(dest/'manifest.json',manifest);refresh_manifest(dest)
   report=kinematics(dest);refresh_manifest(dest)
   results.append(dict(source=source.name,package=name,directory=str(dest),status='kinematics_passed_requires_full_collision_and_visual_review',changes=changes))
  except Exception as exc:results.append(dict(source=source.name,package=name,status='needs_adjustment',error=str(exc)))
 write(WORK/'hand_rebase_result.json',dict(reference_id=ref['id'],results=results,uploaded=False));return results

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
 x=sub.add_parser('capture');x.add_argument('--url',required=True);x.add_argument('--seconds',type=float,default=2);x.add_argument('--hz',type=float,default=10);x.add_argument('--out',type=Path,required=True);x.add_argument('--accepted-initial-pose',action='store_true',required=True)
 x=sub.add_parser('prepare');x.add_argument('--samples',type=Path,required=True);x.add_argument('--id',required=True);x.add_argument('--adjustments',type=Path);x.add_argument('--max-spread-deg',type=float,default=.6);x.add_argument('--max-adjustment-deg',type=float,default=3);x.add_argument('--accepted-initial-pose',action='store_true',required=True)
 for command in ['validate','adopt']:
  x=sub.add_parser(command);x.add_argument('reference',type=Path)
 x=sub.add_parser('import-calibration');x.add_argument('directory',type=Path);x.add_argument('--apply',action='store_true')
 x=sub.add_parser('rebase');x.add_argument('--selection',type=Path,required=True);x.add_argument('--modes',type=Path);x.add_argument('--calibration-targets-reviewed',action='store_true')
 a=p.parse_args()
 if a.cmd=='capture':result=capture(a.url,a.seconds,a.hz,a.out);result=dict(samples=len(result['samples']),raw_file=str(a.out))
 elif a.cmd=='prepare':result=dict(reference=str(prepare(a.samples,a.id,a.adjustments,a.max_spread_deg,a.max_adjustment_deg)))
 elif a.cmd=='validate':result=validate_reference(a.reference)
 elif a.cmd=='adopt':result=adopt(a.reference)
 elif a.cmd=='import-calibration':result=import_calibration(a.directory,a.apply)
 else:result=rebase(a.selection,a.modes,a.calibration_targets_reviewed)
 print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
