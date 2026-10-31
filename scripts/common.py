"""Paths are derived exclusively from this skill folder."""
from pathlib import Path
import hashlib,json,os,sys

SKILL=Path(__file__).resolve().parents[1]
ROBOT=SKILL/'assets/robot'
sys.path.insert(0,str(ROBOT/'scripts'))
WORK=SKILL/'work'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,data):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 temporary=path.with_name('.'+path.name+'.tmp')
 temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 os.replace(temporary,path)
def inputs(directory):return {n:digest(Path(directory)/n) for n in ['fullbody.hdf5','motion.json','fullbody_meta.json']}
def reference_fingerprint():
 return {side:{n:digest(ROBOT/'configs/hand_models'/('orcahand_v1_'+side)/n) for n in ['config.yaml','calibration.yaml']} for side in ['left','right']}
def refresh_manifest(directory):
 p=Path(directory)/'manifest.json';m=read(p)
 m['files']={f.name:digest(f) for f in sorted(Path(directory).iterdir()) if f.is_file() and f.name!='manifest.json'}
 write(p,m)

