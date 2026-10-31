"""Exercise disk-only deployment using real certified ZIPs and temporary fake server paths."""
import copy,io,json,tarfile,tempfile,zipfile
from pathlib import Path
from common import SKILL,ROBOT,WORK,read,write,digest
import deploy,deploy_remote as remote

def run():
 name=read(SKILL/'assets/bundle.json')['reference_examples'][0]
 package=ROBOT/'generated/motion_packages'/name;archive=deploy.verify_archive(package)
 checks=[]
 with tempfile.TemporaryDirectory(prefix='offline_deployment_',dir=WORK) as folder:
  base=Path(folder);selection=base/'local_selection.json';write(selection,[dict(package_path=str(package))])
  assert deploy.upload({},selection)['network_access'] is False
  checks.append('local_dry_run_checks_current_proofs_and_zip_without_network')
  original_guard=remote.guard
  remote.guard=lambda profile:dict(mode='offline_test_fixture',loaded=None)
  try:
   def fixture(case):
    path=base/case;root=path/'replays';root.mkdir(parents=True)
    old='fb_legacy_reference_v1';(root/old).mkdir();(root/old/'original.bin').write_bytes(b'actual-old-recording')
    (root/'fb_unrelated').mkdir();(root/'fb_unrelated'/'keep').write_bytes(b'untouched')
    labels=path/'brain_actions.json';write(labels,{'_metadata':{'keep':True},'old_label':{'rk_action':old,'duration':9.0,'title':'保留标签'},'other':{'rk_action':'fb_unrelated','duration':3}})
    profile=dict(replay_root=str(root),labels_file=str(labels))
    spec=dict(actions=[dict(package=name,replaces=old,duration_s=read(package/'motion.json')['duration_s'],zip_sha256=digest(archive))],label_sha256=digest(labels),old_hashes={old:remote.hashes(root/old)})
    def transfer(spec=spec,zraw=None):
     target=path/'upload.tar'
     with tarfile.open(target,'w') as tar:
      for entry,data in [('selection.json',json.dumps(spec).encode()),(name+'.zip',archive.read_bytes() if zraw is None else zraw)]:
       info=tarfile.TarInfo(entry);info.size=len(data);tar.addfile(info,io.BytesIO(data))
     return target
    return path,root,old,labels,profile,spec,transfer
   for case in ['labels_changed','old_changed','zip_corrupt']:
    path,root,old,labels,profile,spec,transfer=fixture(case)
    tar=transfer()
    if case=='labels_changed':labels.write_bytes(labels.read_bytes()+b' ')
    elif case=='old_changed':(root/old/'original.bin').write_bytes(b'changed')
    else:
     bad=copy.deepcopy(spec);bad['actions'][0]['zip_sha256']='0'*64;tar=transfer(bad)
    before=remote.hashes(root/old);label_before=labels.read_bytes()
    try:remote.install(tar,profile)
    except AssertionError:pass
    else:raise AssertionError('Unsafe stale/corrupt install accepted: '+case)
    assert remote.hashes(root/old)==before and labels.read_bytes()==label_before and not (root/name).exists()
   checks.append('changed_labels_old_recordings_and_corrupt_zip_rejected_before_commit')
   path,root,old,labels,profile,spec,transfer=fixture('complete')
   backup=path/'old.tar';remote.backup(profile,[old],str(backup))
   with tarfile.open(backup) as tar:assert tar.extractfile(old+'/original.bin').read()==b'actual-old-recording'
   result=remote.install(transfer(),profile)
   assert result['status']=='complete' and not result['hardware_commands_sent'] and not result['startup_changed']
   assert not (root/old).exists() and remote.hashes(root/name)==remote.hashes(package)
   assert (root/'fb_unrelated'/'keep').read_bytes()==b'untouched'
   table=read(labels);assert table['old_label']['rk_action']==name and table['old_label']['title']=='保留标签' and table['_metadata']=={'keep':True} and table['other']['rk_action']=='fb_unrelated'
   assert backup.exists() and not list(path.glob('.robot_skill_stage_*'))
   checks.append('real_archive_install_label_migration_exact_replacement_and_backup_preservation')
  finally:remote.guard=original_guard
 return dict(status='passed',checks=checks,network_access=False,robot_commands_sent=False,real_server_tested=False)

if __name__=='__main__':
 result=run();write(WORK/'verification/deployment_test.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))
