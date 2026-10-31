"""Audit the copied URDF/MJCF/mappings and refresh descriptive snapshot provenance."""
import argparse,copy,json
import xml.etree.ElementTree as ET
import numpy as np,mujoco
from common import ROBOT,read,write,digest
from calibrated_hand_pose import CalibratedHandPose

def verify(refresh=False):
 path=ROBOT/'generated/model_report.json';report=read(path)
 assert digest(ROBOT/report['source_urdf'])==report['source_sha256']
 assert digest(ROBOT/report['output_mjcf'])==report['output_sha256']
 xml=ET.parse(ROBOT/report['source_urdf']).getroot()
 model=mujoco.MjModel.from_xml_path(str(ROBOT/report['output_mjcf']))
 mappings={g:read(ROBOT/f'configs/{g}_mapping.json') for g in ['arm','neck','body','hand']}
 count=0
 for group,mapping in mappings.items():
  for row in mapping['joints']:
   name=row['model_joint'];joint=model.joint(name)
   lowhigh=sorted(row['sign']*v+row['offset_rad'] for v in row['real_limits_rad'])
   limit=xml.find(f"joint[@name='{name}']/limit")
   np.testing.assert_allclose(lowhigh,[float(limit.get(k)) for k in ['lower','upper']],rtol=0,atol=1e-6)
   np.testing.assert_allclose(lowhigh,row['model_limits_rad'],rtol=0,atol=1e-6)
   # mj_saveLastXML serializes these angles at limited decimal precision.
   np.testing.assert_allclose(lowhigh,model.jnt_range[joint.id],rtol=0,atol=1e-5)
   count+=1
 cal=CalibratedHandPose()
 if refresh:
  report.setdefault('original_build_mapping_hashes',{g:r['sha256'] for g,r in report['joint_mapping_groups'].items()})
  report.setdefault('original_build_hand_calibration_revision',report.get('hand_calibration_revision'))
  report['arm_mapping_sha256']=digest(ROBOT/'configs/arm_mapping.json')
  for group in mappings:report['joint_mapping_groups'][group]['sha256']=digest(ROBOT/f'configs/{group}_mapping.json')
  report['hand_calibration_revision']=cal.revision
  report['portable_snapshot_audit']=dict(status='passed',joint_count=count,geometry_rebuilt=False,note='URDF, MJCF and all mapping limits numerically audited; source geometry bytes unchanged. Descriptive paths/hash fields normalized for portable snapshot.')
  write(path,report)
 assert report['arm_mapping_sha256']==digest(ROBOT/'configs/arm_mapping.json')
 for group in mappings:assert report['joint_mapping_groups'][group]['sha256']==digest(ROBOT/f'configs/{group}_mapping.json'),group+' model report is stale'
 assert report['hand_calibration_revision']==cal.revision
 return dict(status='passed',mapped_joints=count,geometry_unchanged=True,active_calibration_revision=cal.revision)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-report',action='store_true');a=p.parse_args();print(json.dumps(verify(a.refresh_report),ensure_ascii=False,indent=2))
