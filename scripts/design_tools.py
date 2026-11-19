"""Compose overlapping control-chain phrases from portable, editable design seeds."""
import argparse,copy
from pathlib import Path
import numpy as np
from common import SKILL,read,write


def compose(request):
    bank=read(SKILL/'assets/motion_components.json')['components']
    duration=float(request['duration_s']);tracks={};phrases=[];notes=[];ends={}
    if not np.isfinite(duration) or duration<=0:raise ValueError('Positive finite duration required')
    for entry in sorted(request['components'],key=lambda x:x['start_s']):
        component=bank[entry['component']];start,end=float(entry['start_s']),float(entry['end_s'])
        if not np.isfinite([start,end]).all() or not 0<=start<end<=duration:raise ValueError('Component timing outside action')
        span=end-start;notes.append(component['meaning'])
        for chain,source in component.get('tracks',{}).items():
            if start<=ends.get(chain,-1):raise ValueError('Same-chain components must be separated by a rest interval')
            frames=[]
            for f in source['keyframes']:
                frame=copy.deepcopy(f);frame['t']=start+frame.pop('phase')*span;frames.append(frame)
            row=tracks.setdefault(chain,dict(mode=source['mode'],keyframes=[]))
            if row['mode']!=source['mode']:raise ValueError('Cannot mix relative and absolute components')
            row['keyframes']+=frames;ends[chain]=end
        for p in component.get('hand_phrases',[]):
            phrase=copy.deepcopy(p);phrase['start_s']=start
            for f in phrase['keys']:f['t_s']=f.pop('phase')*span
            if end-(start+phrase['keys'][-1]['t_s'])<.12:raise ValueError('Hand component needs 0.12s stagger settling time')
            phrases.append(phrase)
    for track in tracks.values():
        frames=track['keyframes']
        if frames[0]['t']>0:frames.insert(0,dict(t=0,kind='stop',q={}))
        if frames[-1]['t']<duration:frames.append(dict(t=duration,kind='stop',q={}))
    result=dict(package=request['package'],name=request['name'],duration_s=duration,sample_hz=request.get('sample_hz',100),
        intent=request.get('intent',dict(meaning='；'.join(notes),status='draft_design_requires_inspection')),
        tracks=tracks,hand_phrases=phrases,component_sources=request['components'],notes=notes)
    for key in ['timing','speed_caps_deg_s','extend_for_speed_limits','near_head']:
        if key in request:result[key]=request[key]
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('composition',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();write(a.out,compose(read(a.composition)))
