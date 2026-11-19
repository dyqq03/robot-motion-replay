"""Offline dependency differences and explicit runtime evidence gaps; never contacts a robot."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from common import SKILL, ROBOT, read, write, digest, reference_fingerprint


def snapshot(compare=None,runtime=None):
    files=[*sorted((ROBOT/'configs').glob('*_mapping.json')),
        *sorted((ROBOT/'configs').glob('*_motion_defaults.json')),
        ROBOT/'configs/hand_hardware.json',ROBOT/'configs/design_collision_rules.json',
        ROBOT/'generated/humanoid_shadow.xml',ROBOT/'URDF_90kg_with_hands/urdf/humanoid_with_hands.urdf',
        ROBOT/'backend/fullbody_player.py',ROBOT/'backend/convert_bag.py',SKILL/'assets/server_profile.json',SKILL/'assets/active_robot.json',SKILL/'assets/posture_landmarks.json']
    model=ROBOT/'generated/humanoid_shadow.xml';xml=ET.parse(model).getroot()
    if xml.find('.//include') is not None:raise ValueError('Audit requires a flattened MJCF')
    compiler=xml.find('compiler');options={} if compiler is None else compiler.attrib
    for node in xml.iter():
        if node.tag in ('mesh','texture','hfield') and 'file' in node.attrib:
            folder=options.get('texturedir' if node.tag=='texture' else 'meshdir',options.get('assetdir',''))
            files.append((model.parent/folder/node.attrib['file']).resolve())
    rules=read(ROBOT/'configs/design_collision_rules.json')
    if rules.get('proxy_manifest'):
        manifest=ROBOT/rules['proxy_manifest'];proxy=read(manifest)
        files.extend([manifest,ROBOT/proxy['model']])
        files.extend(ROBOT/group['source_mesh'] for group in proxy['groups'])
    files.extend(sorted((SKILL/'scripts').glob('*.py')))
    files=sorted({p.resolve() for p in files})
    if any(not p.is_relative_to(SKILL) for p in files):raise ValueError('External dependency in skill snapshot')
    result=dict(schema_version=1,active_robot=read(SKILL/'assets/active_robot.json'),
        server_profile=read(SKILL/'assets/server_profile.json'),
        hand_calibration_files_sha256=reference_fingerprint(),
        files_sha256={str(p.relative_to(SKILL)):digest(p) for p in files},network_access=False,
        physical_alignment='not_verified_by_this_command',runtime_status='not_queried')
    if compare:
        old=read(compare);a=old['files_sha256'];b=result['files_sha256']
        result['changed_files']=[p for p in sorted(set(a)|set(b)) if a.get(p)!=b.get(p)]
        result['calibration_changed']=old.get('hand_calibration_files_sha256')!=result['hand_calibration_files_sha256']
    if runtime:
        data=read(runtime)
        if data.get('host')!=result['server_profile']['host']:raise ValueError('Runtime evidence belongs to a different target; select and align the intended profile first')
        if not data.get('observed_at_utc') or not data.get('source'):raise ValueError('Runtime evidence needs observation time and actual source')
        result['runtime_evidence']=data;result['runtime_status']='imported_evidence_not_live_verification'
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--compare',type=Path);p.add_argument('--runtime-evidence',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();write(a.out,snapshot(a.compare,a.runtime_evidence))
