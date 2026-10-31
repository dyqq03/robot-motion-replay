"""Remote disk-only helpers; never start a stack, enable motors, or play actions."""
from pathlib import Path
import hashlib,json,os,stat,shutil,subprocess,shlex,urllib.request,urllib.error,tarfile,zipfile,io,base64,uuid

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(d):
 d=Path(d);assert d.is_dir() and not d.is_symlink()
 assert not any(p.is_symlink() for p in d.rglob('*'))
 return {str(p.relative_to(d)):sha(p) for p in d.rglob('*') if p.is_file()}
def guard(profile):
 try:
  with urllib.request.urlopen(profile['api']+'/api/snapshot',timeout=4) as r:h=json.load(r)['health']
 except urllib.error.URLError:
  active=[]
  for p in Path('/proc').iterdir():
   if not p.name.isdigit():continue
   try:args=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   except OSError:continue
   if any(any(n in a for n in ['openarm_ui','monitor_node','body_action_server','fullbody_player','ker_teleop','hand_controller_node','ros2_control_node','neck_node']) for a in args[:3]):active.append(p.name)
  assert not active,'API unavailable while control processes remain; cannot confirm idle'
  return dict(mode='stack_stopped')
 assert not any(h['fb'].get(k) for k in ['recording','preparing']) and not h['teach']['on'] and not h['teach'].get('recording')
 assert not any(h['fbplay'].get(k) for k in ['playing','approaching','paused']),'Replay active'
 code='''import rclpy
from body_actions.srv import QueryBodyState
rclpy.init();n=rclpy.create_node('readonly_skill_upload_guard');c=n.create_client(QueryBodyState,'/body/query_state')
try:
 assert c.wait_for_service(timeout_sec=4.)
 f=c.call_async(QueryBodyState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=4.);assert f.done()
 r=f.result();assert not r.busy and r.state=='idle',(r.busy,r.state)
 assert not any('ker_teleop' in x.lower() for x,ns in n.get_node_names_and_namespaces())
finally:n.destroy_node();rclpy.shutdown()
'''
 r=subprocess.run(['bash','-c','. '+shlex.quote(profile['environment_script'])+'\npython3 -c '+shlex.quote(code)],capture_output=True,text=True,timeout=15)
 assert r.returncode==0,r.stdout+r.stderr
 return dict(mode='http_and_body_idle',loaded=h['fbplay'].get('name'))

def snapshot(profile,olds):
 root=Path(profile['replay_root']);label=Path(profile['labels_file'])
 instructions={str(p):p.read_text() for p in [Path('/AGENTS.md'),*reversed([x/'AGENTS.md' for x in root.parents]),root/'AGENTS.md'] if p.is_file()}
 return dict(instructions=instructions,labels_base64=base64.b64encode(label.read_bytes()).decode(),label_sha256=sha(label),packages=sorted(p.name for p in root.iterdir() if p.is_dir()),old_hashes={n:hashes(root/n) for n in olds},free_bytes=shutil.disk_usage(root).free)

def backup(profile,olds,target):
 root=Path(profile['replay_root'])
 with tarfile.open(target,'w') as tar:
  for name in olds:
   assert Path(name).name==name and name.startswith('fb_');hashes(root/name)
   tar.add(root/name,arcname=name)
 return dict(archive=target,sha256=sha(target))

def install(transfer,profile):
 root=Path(profile['replay_root']);labels=Path(profile['labels_file']);before_guard=guard(profile)
 with tarfile.open(transfer) as tar:
  spec=json.load(tar.extractfile('selection.json'));rows=spec['actions'];expected={'selection.json'}|{r['package']+'.zip' for r in rows}
  assert set(tar.getnames())==expected and len(tar.getmembers())==len(expected)
  raw=labels.read_bytes();assert sha(labels)==spec['label_sha256'],'Labels changed since local backup'
  for r in rows:
   if r.get('replaces'):assert hashes(root/r['replaces'])==spec['old_hashes'][r['replaces']]
  # Stage beside the replay root: /tmp may be a different filesystem.
  stage=root.parent/('.robot_skill_stage_'+uuid.uuid4().hex);stage.mkdir();ready=[]
  for row in rows:
   name=row['package'];assert Path(name).name==name and name.startswith('fb_')
   zraw=tar.extractfile(name+'.zip').read();assert hashlib.sha256(zraw).hexdigest()==row['zip_sha256']
   dest=stage/name;dest.mkdir()
   with zipfile.ZipFile(io.BytesIO(zraw)) as z:
    manifest=json.loads(z.read(name+'/manifest.json'));names={name+'/'+n for n in manifest['files']}|{name+'/manifest.json'}
    assert set(z.namelist())==names and len(z.namelist())==len(names) and z.testzip() is None
    for item in z.namelist():
     filename=item.removeprefix(name+'/');assert Path(filename).name==filename
     data=z.read(item)
     if filename!='manifest.json':assert hashlib.sha256(data).hexdigest()==manifest['files'][filename]
     (dest/filename).write_bytes(data)
   assert json.loads((dest/'collision_validation.json').read_text())['status']=='passed'
   if (root/name).exists():assert hashes(root/name)==hashes(dest),'Different existing version'
   ready.append((row,dest,hashes(dest)))
  commit_guard=guard(profile);assert labels.read_bytes()==raw
  mapping={r['replaces']:r for r in rows if r.get('replaces')};table=json.loads(raw);migrated=[]
  for k,v in table.items():
   if not k.startswith('_') and isinstance(v,dict) and v.get('rk_action') in mapping:
    row=mapping[v['rk_action']];v['rk_action']=row['package'];v['duration']=row['duration_s'];migrated.append(k)
  loaded=commit_guard.get('loaded');assert loaded not in mapping,'Old package cached: load an unaffected package first, then retry; do not play'
  for row,dest,hh in ready:
   final=root/row['package']
   if final.exists():shutil.rmtree(dest)
   else:os.rename(dest,final)
   assert hashes(final)==hh
  if migrated:
   tmp=labels.with_name('.'+labels.name+'.skill_upload.tmp');assert not tmp.exists();attr=labels.stat()
   with tmp.open('x') as f:json.dump(table,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
   os.chmod(tmp,stat.S_IMODE(attr.st_mode));os.chown(tmp,attr.st_uid,attr.st_gid);assert labels.read_bytes()==raw;os.replace(tmp,labels)
  removed=[]
  for old in mapping:
   assert hashes(root/old)==spec['old_hashes'][old];shutil.rmtree(root/old);removed.append(old)
  stage.rmdir();Path(transfer).unlink()
  return dict(status='complete',uploaded=[r['package'] for r in rows],removed=removed,migrated_labels=migrated,guards=[before_guard,commit_guard],hardware_commands_sent=False,startup_changed=False,label_sha256=sha(labels))
