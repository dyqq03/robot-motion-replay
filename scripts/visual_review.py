"""Evidence preparation and explicit human/assistant critique, never automatic aesthetics."""
import argparse
import datetime
from pathlib import Path
import numpy as np
from common import read, write, inputs, digest, refresh_manifest, SKILL

STATUSES = ('pending', 'needs_changes', 'passed')


def prepare(directory, times=None):
    import cv2
    from PIL import Image, ImageDraw, ImageFont
    directory = Path(directory)
    duration = read(directory/'motion.json')['duration_s']
    times = list(times) if times is not None else np.linspace(0, duration, 12).tolist()
    if not times or not np.isfinite(times).all() or min(times) < 0 or max(times) > duration:
        raise ValueError('Review times must lie inside the exported clock')
    cap = cv2.VideoCapture(str(directory/'preview.mp4'))
    fps, count = cap.get(cv2.CAP_PROP_FPS), int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if not cap.isOpened() or fps <= 0 or count < 1:
        raise ValueError('A decodable preview.mp4 is required')
    frames, actual = [], []
    try:
        for ts in times:
            index = min(count-1, round(float(ts)*fps))
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, rgb = cap.read()
            if not ok: raise ValueError('Cannot decode review frame at '+str(ts))
            frames.append(Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)))
            actual.append(index/fps)
    finally:
        cap.release()
    w, h = frames[0].size
    sheet = Image.new('RGB', (w*2, (h+28)*int(np.ceil(len(frames)/2))), '#171b22')
    font = ImageFont.truetype(str(SKILL/'assets/fonts/NotoSansCJK-Regular.ttc'), 18)
    for i, (frame, ts) in enumerate(zip(frames, actual)):
        x, y = i%2*w, i//2*(h+28)
        sheet.paste(frame, (x,y+28))
        ImageDraw.Draw(sheet).text((x+8,y+2), f'{ts:.3f}s', font=font, fill='white')
    sheet.save(directory/'review_sheet.jpg', quality=93)
    data = dict(schema_version=1, status='pending', requested_times_s=times,
        actual_times_s=actual, input_sha256=inputs(directory),
        files_sha256={n:digest(directory/n) for n in ['preview.mp4','review_sheet.jpg','design.json'] if (directory/n).is_file()},
        note='Decoded samples prepared for inspection; no visual judgment performed.')
    write(directory/'review_evidence.json', data)
    # Preparing a new sheet invalidates an older review rather than borrowing it.
    (directory/'visual_review.json').unlink(missing_ok=True)
    refresh_manifest(directory)
    return data


def record(directory, notes, reviewer, status='pending', assessment=None):
    directory = Path(directory)
    if status not in STATUSES or not reviewer.strip(): raise ValueError('Explicit reviewer and review status required')
    evidence = read(directory/'review_evidence.json')
    if evidence['input_sha256'] != inputs(directory): raise ValueError('Review samples are stale')
    for name, h in evidence['files_sha256'].items():
        if digest(directory/name) != h: raise ValueError('Review media changed: '+name)
    data = read(assessment) if isinstance(assessment, (str, Path)) else (assessment or {})
    observations = data.get('observations', [])
    duration = read(directory/'motion.json')['duration_s']
    for row in observations:
        ts = row.get('time_s')
        if isinstance(ts,bool) or not isinstance(ts,(float,int)) or not np.isfinite(ts) or not 0 <= ts <= duration:
            raise ValueError('Each observation needs an exported time_s')
        if not row.get('aspect','').strip() or not row.get('observation','').strip():
            raise ValueError('Each observation needs aspect and concrete observation')
        if row.get('verdict') not in ('acceptable','needs_changes'):
            raise ValueError('Observation verdict must be acceptable or needs_changes')
    viewed = data.get('viewed_files', [])
    for name in viewed:
        if name not in ('review_sheet.jpg','preview.mp4','keyframes.jpg'): raise ValueError('Viewed files must name prepared images or video')
        if not (directory/name).is_file():raise ValueError('Missing viewed file: '+name)
    if status == 'passed':
        if not notes.strip() or not observations or not viewed or not data.get('method','').strip():
            raise ValueError('Passing review requires method, viewed files and timed observations')
        if any(row['verdict'] != 'acceptable' for row in observations):
            raise ValueError('Unresolved observations cannot pass')
    media = {name:digest(directory/name) for name in set(viewed)|set(evidence['files_sha256'])|{'review_evidence.json'}}
    report = dict(schema_version=2, status=status, reviewer=reviewer, notes=notes,
        method=data.get('method','Not yet inspected'), observations=observations,
        viewed_files=viewed, input_sha256=inputs(directory)|media,
        inspection_is_self_reported=True, automatic_aesthetic_judgment=False,
        user_acceptance='not_recorded', physical_validation='not_recorded',
        recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    history_path=directory/'review_history.json'
    history=read(history_path) if history_path.exists() else []
    history.append(report);write(history_path,history)
    write(directory/'visual_review.json', report); refresh_manifest(directory)
    return report


def compare(before, after, output, fractions=(0,.2,.4,.6,.8,1)):
    """Same camera layouts, paired normalized-clock frames; labels show actual time."""
    import cv2
    from PIL import Image, ImageDraw
    rows = []
    for fraction in fractions:
        if not np.isfinite(fraction) or not 0 <= fraction <= 1: raise ValueError('Fractions must be in [0,1]')
        pair=[]
        for directory in (Path(before),Path(after)):
            cap=cv2.VideoCapture(str(directory/'preview.mp4'))
            try:
                count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
                index=round(fraction*(count-1));cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,rgb=cap.read()
                if not ok or fps<=0:raise ValueError('Cannot decode comparison video')
                img=Image.fromarray(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB))
                row=Image.new('RGB',(img.width,img.height+28),'#171b22');row.paste(img,(0,28))
                ImageDraw.Draw(row).text((8,8),f'{directory.name} | {index/fps:.3f}s | phase {fraction:.2f}',fill='white')
                pair.append(row)
            finally:cap.release()
        if pair[0].size!=pair[1].size:raise ValueError('Comparison requires matching view layouts')
        rows.append(pair)
    if not rows:raise ValueError('At least one comparison phase required')
    w,h=rows[0][0].size;out=Image.new('RGB',(w*2,h*len(rows)))
    for i,pair in enumerate(rows):
        for j,img in enumerate(pair):out.paste(img,(j*w,i*h))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);out.save(output)
    write(output.with_suffix('.json'),dict(before=inputs(before),after=inputs(after),fractions=list(fractions),alignment='normalized clock; not semantic event matching'))
    return dict(image=str(output))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
    x=sub.add_parser('prepare');x.add_argument('package',type=Path);x.add_argument('--times',type=float,nargs='+')
    x=sub.add_parser('compare');x.add_argument('before',type=Path);x.add_argument('after',type=Path);x.add_argument('--out',type=Path,required=True);x.add_argument('--fractions',type=float,nargs='+',default=[0,.2,.4,.6,.8,1])
    a=p.parse_args()
    if a.cmd=='prepare':prepare(a.package,a.times)
    else:compare(a.before,a.after,a.out,a.fractions)
