"""Same display name, preserved local bytes and content-addressed revision receipts."""
import datetime, os, shutil, tempfile, uuid, zipfile
from pathlib import Path
from common import SKILL, WORK, digest, write


def archive_matches(directory,archive):
    import hashlib
    directory,archive=Path(directory),Path(archive)
    if any(p.is_symlink() or p.is_dir() for p in directory.iterdir()):raise ValueError('Flat regular package files required')
    expected={directory.name+'/'+p.name:digest(p) for p in directory.iterdir() if p.is_file()}
    with zipfile.ZipFile(archive) as z:
        if set(z.namelist())!=set(expected) or len(z.namelist())!=len(expected):raise ValueError('Archive contents differ from package')
        if any(hashlib.sha256(z.read(n)).hexdigest()!=h for n,h in expected.items()):raise ValueError('Archive bytes differ from package')


def replace_local(source,target):
    source,target=Path(source).resolve(),Path(target).resolve();oldzip=target.with_suffix('.zip')
    if source==target or not target.is_relative_to(WORK.resolve()) or not source.is_relative_to(SKILL):
        raise ValueError('Replace a separate work package from a skill-local draft')
    if source.name!=target.name or not target.is_dir() or not oldzip.is_file():raise ValueError('Existing same-name package and ZIP required')
    archive_matches(target,oldzip)
    old_hash=digest(oldzip)
    backup=WORK/'revisions'/target.name/(datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_')+old_hash[:12]+'_'+uuid.uuid4().hex[:8])
    backup.mkdir(parents=True)
    with tempfile.TemporaryDirectory(prefix='.publish_',dir=target.parent) as folder:
        staged=Path(folder)/target.name;shutil.copytree(source,staged);newzip=staged.with_suffix('.zip')
        with zipfile.ZipFile(newzip,'x',zipfile.ZIP_DEFLATED) as z:
            for file in sorted(staged.iterdir()):
                if file.is_file():z.write(file,Path(target.name)/file.name)
        archive_matches(staged,newzip);new_hash=digest(newzip)
        moved=[]
        try:
            for old in (target,oldzip):
                os.rename(old,backup/old.name);moved.append(old)
            os.rename(staged,target);os.rename(newzip,oldzip)
            archive_matches(target,oldzip)
        except BaseException:
            for old in reversed(moved):
                if old.exists():
                    if old.is_dir():shutil.rmtree(old)
                    else:old.unlink()
                os.rename(backup/old.name,old)
            raise
    receipt=dict(package=str(target),archive=str(oldzip),old_zip_sha256=old_hash,zip_sha256=new_hash,
        backup=str(backup),display_name_unchanged=True,remote_changed=False)
    write(backup/'revision.json',receipt)
    return receipt
