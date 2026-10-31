"""Explicit disk-only upload; credentials via SSH agent/key or environment, never stored."""
import argparse,base64,json,os,shlex,sys,tarfile,tempfile,uuid,zipfile,shutil
from pathlib import Path
from common import SKILL,WORK,read,write,digest

def interact(args):
 import pexpect
 p=pexpect.spawn(args[0],args[1:],encoding='utf-8',timeout=900,maxread=2**22)
 while True:
  event=p.expect(['[Pp]assword:',pexpect.EOF,pexpect.TIMEOUT])
  if event==0:
   password=os.environ.get('ROBOT_SSH_PASSWORD')
   if password is None:p.close(force=True);raise ValueError('Use an SSH key/agent or set ROBOT_SSH_PASSWORD in the calling environment')
   p.sendline(password)
  elif event==1:break
  else:p.close(force=True);raise TimeoutError('SSH/SCP timed out; keep receipt and retry without deleting unknown files')
 text=p.before.strip();p.close()
 if p.exitstatus!=0:raise RuntimeError(text)
 return text
def ssh(profile,code):
 encoded=base64.b64encode(code.encode()).decode();remote='python3 -c '+shlex.quote('import base64;exec(compile(base64.b64decode('+repr(encoded)+'),"<portable-skill>","exec"))')
 args=['ssh','-p',str(profile['port']),'-o','ConnectTimeout=12','-o','StrictHostKeyChecking=accept-new']
 if profile.get('bind_interface'):args+=['-B',profile['bind_interface']]
 return interact(args+[profile['user']+'@'+profile['host'],remote])
def scp(profile,source,dest,download=False):
 args=['scp','-P',str(profile['port']),'-o','ConnectTimeout=12','-o','StrictHostKeyChecking=accept-new']
 if profile.get('bind_interface'):args+=['-o','BindInterface='+profile['bind_interface']]
 host=profile['user']+'@'+profile['host']+':'
 return interact(args+([host+source,dest] if download else [source,host+dest]))
def remote_call(profile,expression):
 source=(SKILL/'scripts/deploy_remote.py').read_text()+'\nprint(json.dumps('+expression+',ensure_ascii=False))\n'
 return json.loads(ssh(profile,source))
def verify_archive(directory):
 from motion import verify_export
 verify_export(directory);archive=directory.with_suffix('.zip')
 with zipfile.ZipFile(archive) as z:
  expected={directory.name+'/'+p.name:digest(p) for p in directory.iterdir() if p.is_file()}
  assert set(z.namelist())==set(expected) and len(z.namelist())==len(expected)
  import hashlib
  for name,h in expected.items():assert hashlib.sha256(z.read(name)).hexdigest()==h
 return archive
def upload(profile,selection,apply=False):
 rows=[]
 for item in read(selection):
  directory=Path(item['package_path']).resolve()
  if not directory.is_relative_to(SKILL):raise ValueError('Copy packages inside this skill before deployment')
  archive=verify_archive(directory);old=item.get('replaces')
  if old is not None and (Path(old).name!=old or not old.startswith('fb_')):raise ValueError('Invalid exact old package name')
  rows.append(dict(package=directory.name,name=read(directory/'motion.json')['name'],duration_s=read(directory/'motion.json')['duration_s'],replaces=old,zip_sha256=digest(archive),archive=str(archive)))
 assert rows and len({r['package'] for r in rows})==len(rows)
 olds=[r['replaces'] for r in rows if r['replaces']];assert len(set(olds))==len(olds) and not set(olds)&{r['package'] for r in rows}
 if not apply:return dict(status='dry_run_local_only',actions=rows,network_access=False)
 token=uuid.uuid4().hex;out=WORK/'deployments'/token;out.mkdir(parents=True)
 snapshot=remote_call(profile,'snapshot('+repr(profile)+','+repr(olds)+')');write(out/'server_before.json',snapshot)
 if snapshot['instructions']:raise ValueError('Remote AGENTS instructions found; read saved server_before.json and adapt deployment before mutation')
 (out/'brain_actions.before.json').write_bytes(base64.b64decode(snapshot['labels_base64']))
 remote_backup='/tmp/robot_skill_backup_'+token+'.tar'
 if olds:
  result=remote_call(profile,'backup('+repr(profile)+','+repr(olds)+','+repr(remote_backup)+')')
  local=out/'old_packages.tar';scp(profile,remote_backup,str(local),True);assert digest(local)==result['sha256']
  with tarfile.open(local) as tar:
   for item in tar.getmembers():
    dest=(out/'old_packages'/item.name).resolve()
    assert dest.is_relative_to((out/'old_packages').resolve()) and not item.issym() and not item.islnk()
   tar.extractall(out/'old_packages',filter='data')
  from deploy_remote import hashes
  for old in olds:assert hashes(out/'old_packages'/old)==snapshot['old_hashes'][old]
  ssh(profile,'from pathlib import Path\nPath('+repr(remote_backup)+').unlink()\nprint("backup downloaded")')
 spec=dict(actions=rows,old_hashes=snapshot['old_hashes'],label_sha256=snapshot['label_sha256']);write(out/'selection.json',spec)
 transfer=out/'upload.tar'
 with tarfile.open(transfer,'w') as tar:
  tar.add(out/'selection.json',arcname='selection.json')
  for row in rows:tar.add(row['archive'],arcname=row['package']+'.zip')
 if snapshot['free_bytes']<transfer.stat().st_size*3:raise ValueError('Insufficient server staging space')
 remote_tar='/tmp/robot_skill_upload_'+token+'.tar';scp(profile,str(transfer),remote_tar)
 result=remote_call(profile,'install('+repr(remote_tar)+','+repr(profile)+')');write(out/'receipt.json',result)
 return dict(receipt=str(out/'receipt.json'),**result)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',type=Path,default=SKILL/'assets/server_profile.json');sub=p.add_subparsers(dest='cmd',required=True)
 sub.add_parser('inspect');x=sub.add_parser('upload');x.add_argument('selection',type=Path);x.add_argument('--apply',action='store_true');a=p.parse_args();profile=read(a.profile)
 result=remote_call(profile,'snapshot('+repr(profile)+',[])') if a.cmd=='inspect' else upload(profile,a.selection,a.apply)
 print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
