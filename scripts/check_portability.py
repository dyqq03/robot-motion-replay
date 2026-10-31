"""Copy this skill, forbid reads of the source folder, and exercise it from a different cwd."""
import argparse,json,os,shutil,subprocess,sys,tempfile
from pathlib import Path
from common import SKILL,WORK,write,read

def run(full=False,forbid=()):
 checks=[];output=[]
 with tempfile.TemporaryDirectory(prefix='robot_motion_portability_') as folder:
  parent=Path(folder);copied=parent/SKILL.name
  shutil.copytree(SKILL,copied,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.venv','venv','.git'))
  assert not any(p.is_symlink() for p in copied.rglob('*'))
  env=os.environ.copy();env.pop('PYTHONPATH',None);env['MUJOCO_GL']='egl'
  env['SKILL_FORBID_READ_ROOT']=os.pathsep.join([str(SKILL),*map(str,forbid)])
  # Install the read/network guard before importing ANY copied skill module.
  bootstrap='''import os,sys,runpy
from pathlib import Path
roots=[Path(x).resolve() for x in os.environ['SKILL_FORBID_READ_ROOT'].split(os.pathsep)]
def audit(event,args):
 if event in ['socket.connect','socket.getaddrinfo']:raise AssertionError('Portability test attempted network access')
 if event=='open' and isinstance(args[0],(str,bytes)):
  p=Path(os.fsdecode(args[0])).resolve()
  if any(p==r or p.is_relative_to(r) for r in roots):raise AssertionError('Read original resource: '+str(p))
sys.addaudithook(audit)
target=sys.argv[1];sys.argv=sys.argv[1:];sys.path.insert(0,str(Path(target).parent));runpy.run_path(target,run_name='__main__')
'''
  commands=[('self_test',[sys.executable,str(copied/'scripts/self_test.py'),*(['--full'] if full else []),'--report',str(copied/'work/verification/relocated_self_test.json')]),('deployment',[sys.executable,str(copied/'scripts/test_deployment.py')]),('manifest',[sys.executable,str(copied/'scripts/bundle_manifest.py')])]
  for label,cmd in commands:
   result=subprocess.run([cmd[0],'-c',bootstrap,*cmd[1:]],cwd=parent,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=600)
   output.append(label+'\n'+result.stdout)
   (WORK/'verification').mkdir(parents=True,exist_ok=True);(WORK/'verification/portability.log').write_text('\n'.join(output))
   if result.returncode:raise RuntimeError('Relocated '+label+' failed; see work/verification/portability.log')
   checks.append(label);print('Relocated '+label+' passed',flush=True)
  details=read(copied/'work/verification/relocated_self_test.json')
 result=dict(status='passed',full=full,checks=checks,copied_to_different_directory=True,run_from_different_cwd=True,original_skill_reads_blocked_by_python_audit=True,external_project_roots_also_blocked=len(forbid),symlinks=False,network_access=False,robot_commands_sent=False,self_test=details)
 write(WORK/'verification/relocation.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--full',action='store_true');p.add_argument('--forbid-root',type=Path,action='append',default=[]);a=p.parse_args();print(json.dumps(run(a.full,a.forbid_root),ensure_ascii=False,indent=2))
