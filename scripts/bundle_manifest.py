"""Versioned hashes for the portable skill; mutable work outputs are excluded."""
import argparse,json
from pathlib import Path
from common import SKILL,read,write,digest
PATH=SKILL/'assets/skill_manifest.json'
def files():
 excluded={'work','__pycache__','.git','.venv','venv','.pytest_cache','.mypy_cache'}
 return {str(p.relative_to(SKILL)):digest(p) for p in sorted(SKILL.rglob('*')) if p.is_file() and p!=PATH and not excluded.intersection(p.relative_to(SKILL).parts) and p.suffix!='.pyc'}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--write',action='store_true');a=p.parse_args();actual=files()
 if a.write:write(PATH,dict(schema_version=1,files=actual,scope='Fixed skill code, model, calibration, examples and references; work outputs excluded'));print('Wrote',len(actual),'file hashes')
 else:
  expected=read(PATH)['files'];changed=sorted(k for k in set(actual)|set(expected) if actual.get(k)!=expected.get(k))
  if changed:raise SystemExit('Bundle changes: '+json.dumps(changed,ensure_ascii=False))
  print('Portable bundle verified:',len(actual),'files')
