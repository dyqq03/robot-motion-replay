"""Local action catalog; review, user approval, physical tests and deployment stay separate."""
import argparse, datetime, hashlib, json, re
from pathlib import Path
from common import WORK, SKILL, read, write, inputs, digest


def content_id(directory):
    return hashlib.sha256(json.dumps(inputs(directory),sort_keys=True).encode()).hexdigest()


def record(package, kind, evidence, database=None):
    if kind not in ('user_accepted','physical_tested','withdrawn'):raise ValueError('Unsupported lifecycle event')
    if not evidence.strip():raise ValueError('Record the actual user message or test evidence source')
    package=Path(package).resolve()
    if not package.is_relative_to(SKILL):raise ValueError('Package must be skill-local')
    path=Path(database) if database else WORK/'catalog/events.json'
    events=read(path) if path.exists() else []
    entry=dict(kind=kind,package=package.name,content_id=content_id(package),evidence=evidence,
        recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),attestation='Operator recorded; not independently verified by this command')
    events.append(entry);write(path,events);return entry


def build(roots=None, database=None, deployments=None):
    roots=roots or [WORK/'packages']
    db=Path(database) if database else WORK/'catalog/events.json'
    events=read(db) if db.exists() else []
    receipts=[]
    for p in sorted((Path(deployments) if deployments else WORK/'deployments').glob('*/receipt.json')):
        data=read(p)
        if data.get('status')=='complete':receipts.append((p,data))
    rows=[]
    for root in map(Path,roots):
        for directory in sorted(root.iterdir()) if root.exists() else []:
            if not directory.is_dir() or not (directory/'motion.json').is_file():continue
            m=read(directory/'motion.json');identity=content_id(directory)
            related=[e for e in events if e['package']==directory.name and e['content_id']==identity]
            visual=read(directory/'visual_review.json') if (directory/'visual_review.json').is_file() else {}
            visual_current=bool(visual) and set(inputs(directory))<=set(visual.get('input_sha256',{})) and all((directory/n).is_file() and digest(directory/n)==h for n,h in visual.get('input_sha256',{}).items())
            hdf=digest(directory/'fullbody.hdf5');matches=[]
            for path,receipt in receipts:
                for item in receipt.get('installed',[]):
                    if item['package']==directory.name and all(item.get('files_sha256',{}).get(k)==v for k,v in inputs(directory).items()):
                        matches.append(dict(receipt=str(path.relative_to(SKILL)),target=receipt.get('target'),note='Historical upload receipt, current remote presence not queried'))
            match=re.fullmatch(r'(.*)_v(\d+)',directory.name)
            rows.append(dict(package=directory.name,name=m['name'],action_id=m.get('action_id',match[1] if match else directory.name),
                version=int(match[2]) if match else None,content_id=identity,duration_s=m['duration_s'],robot_id=m.get('robot_id'),
                hand_reference_id=m.get('hand_baseline_id'),arm_reference_id=m.get('baseline_id'),
                controls=dict(arms=m.get('arms',[]),neck=bool(m.get('neck_joint_names')),hands=list(m.get('hand_joint_names',{}))),
                package_path=str(directory.relative_to(SKILL)),video_path=str((directory/'preview.mp4').relative_to(SKILL)) if (directory/'preview.mp4').exists() else None,
                assistant_review=visual.get('status','pending') if visual_current else 'stale_or_missing',
                user_accepted=any(e['kind']=='user_accepted' for e in related),physical_tested=any(e['kind']=='physical_tested' for e in related),
                withdrawn=bool(related and related[-1]['kind']=='withdrawn'),upload_receipts=matches,current_remote_presence='not_queried'))
    for row in rows:
        peers=[r for r in rows if r['action_id']==row['action_id'] and r['robot_id']==row['robot_id'] and not r['withdrawn']]
        row['highest_named_version']=not row['withdrawn'] and row['version']==max([r['version'] for r in peers if r['version'] is not None],default=None)
    return dict(schema_version=1,actions=rows,count=len(rows),latest_is_name_based_only=True,network_access=False)


def save(result,out):
    out=Path(out);write(out.with_suffix('.json'),result)
    def cell(v):return str(v).replace('|','\\|').replace('\n',' ')
    lines=['# 本地动作目录','',f"共 {result['count']} 个目录。上传回执仅表示历史上传，当前服务器状态未查询。",'',
        '|动作|文件名|时长|机器人|视觉审阅|用户认可|真机测试|历史上传回执|','|---|---|---:|---|---|---|---|---|']
    for row in result['actions']:
        lines.append('|'+ '|'.join(map(cell,[row['name'],row['package'],row['duration_s'],row['robot_id'],row['assistant_review'],row['user_accepted'],row['physical_tested'],len(row['upload_receipts'])]))+'|')
    out.with_suffix('.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
    x=sub.add_parser('build');x.add_argument('--roots',type=Path,nargs='+');x.add_argument('--out',type=Path,default=WORK/'catalog/actions')
    x=sub.add_parser('record');x.add_argument('package',type=Path);x.add_argument('--kind',choices=['user_accepted','physical_tested','withdrawn'],required=True);x.add_argument('--evidence',required=True)
    a=p.parse_args()
    if a.cmd=='build':save(build(a.roots),a.out)
    else:record(a.package,a.kind,a.evidence)
