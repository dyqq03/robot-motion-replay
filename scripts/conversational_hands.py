"""Independent, C2 conversational hand-shape phrases in calibrated degrees."""
import numpy as np
def smooth(x):
 x=np.clip(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x))

def shapes(d,side):
 ns=d.hand_names[side]
 def delta(entries):
  out=np.zeros(16)
  for n,v in entries.items():out[ns.index(n)]=v
  return out
 opening={}
 for f,m,p,a in [('index',-2,-2,-6),('middle',-3,-4,0),('ring',-2,-3,3),('pinky',-1,-2,9)]:
  opening.update({f+'_mcp':m,f+'_pip':p,f+'_abd':a})
 opening.update(thumb_abd=-3,thumb_mcp=-2 if side=='left' else -5,thumb_pip=-4,thumb_dip=-2)
 curl={}
 for f,s in [('index',.70),('middle',.90),('ring',1.),('pinky',.78)]:
  curl[f+'_mcp']=24*s;curl[f+'_pip']=29*s
 curl.update(middle_abd=-.3,ring_abd=.3,pinky_abd=4 if side=='left' else 1,thumb_dip=4,thumb_pip=2)
 opened=delta(opening);curled=delta(curl)
 # Soft partial fold: retains more index extension, and less ring flexion.
 soft=curled*.64
 soft[ns.index('index_mcp')]*=.6;soft[ns.index('index_pip')]*=.65
 soft[ns.index('pinky_abd')]=4 if side=='left' else 2
 return dict(rest=np.zeros(16),open=opened,curl=curled,soft=soft,broad=opened*.7,relaxed=curled*.25)

# Each episode is side,start,[(time offset,shape,amplitude)...]. Unequal timing,
# holds and finger offsets avoid repeated bilateral "pumping".
EPISODES={
 1:[('right',.65,[(0,'rest',1),(1.25,'open',.85),(3.05,'curl',.85),(5.45,'rest',1)]),
    ('left',3.15,[(0,'rest',1),(1.55,'soft',.85),(3.45,'rest',1)]),
    ('right',9.35,[(0,'rest',1),(1.55,'broad',1),(3.45,'relaxed',1),(6.0,'rest',1)]),
    ('left',13.65,[(0,'rest',1),(1.35,'open',.70),(3.0,'soft',1),(5.1,'rest',1)])],
 2:[('left',.75,[(0,'rest',1),(1.3,'open',.9),(3.1,'curl',.9),(5.65,'rest',1)]),
    ('right',6.1,[(0,'rest',1),(1.55,'soft',1),(3.9,'rest',1)]),
    ('left',12.25,[(0,'rest',1),(1.5,'soft',.9),(3.35,'open',.8),(6.1,'rest',1)])],
 3:[('left',.7,[(0,'rest',1),(1.35,'broad',.9),(3.1,'soft',.9),(5.6,'rest',1)]),
    ('right',1.8,[(0,'rest',1),(1.65,'curl',.85),(3.6,'open',.75),(5.85,'rest',1)]),
    ('right',10.75,[(0,'rest',1),(1.5,'open',.9),(3.5,'soft',1),(6.0,'rest',1)]),
    ('left',14.0,[(0,'rest',1),(1.4,'relaxed',1.4),(4.3,'rest',1)])],
 4:[('left',1.0,[(0,'rest',1),(1.5,'curl',.85),(3.45,'open',.75),(5.9,'rest',1)]),
    ('right',6.7,[(0,'rest',1),(1.4,'open',.9),(3.25,'curl',.85),(5.25,'soft',.8),(7.9,'rest',1)]),
    ('left',15.35,[(0,'rest',1),(1.25,'broad',.8),(2.65,'relaxed',1.2),(4.4,'rest',1)])],
 5:[('right',.65,[(0,'rest',1),(1.35,'soft',.95),(3.35,'open',.7),(5.45,'rest',1)]),
    ('left',1.6,[(0,'rest',1),(1.45,'open',.65),(3.25,'relaxed',1.3),(4.8,'rest',1)]),
    ('right',8.85,[(0,'rest',1),(1.45,'open',.9),(3.3,'curl',.85),(5.65,'rest',1)]),
    ('left',14.3,[(0,'rest',1),(1.35,'soft',1),(3.0,'broad',.75),(4.95,'rest',1)]),
    ('right',16.0,[(0,'rest',1),(1.4,'relaxed',1.4),(3.4,'rest',1)])]
}

def apply_phrases(d,t,q,episodes):
 out=q.copy();timeline=[]
 for side in {x[0] for x in episodes}:
  off=14 if side=='left' else 30;out[:,off:off+16]=d.rest[off:off+16]
 for side,start,keys in episodes:
  off=14 if side=='left' else 30;bank=shapes(d,side);ns=d.hand_names[side]
  # Keep ABD spacing coherent; stagger flexion by only 40–120ms per digit.
  delays={'index':0.,'middle':.04,'ring':.08,'pinky':.12,'thumb':.10}
  ts=np.array([start+k[0] for k in keys]);values=np.array([bank[k[1]]*k[2] for k in keys])
  for j,n in enumerate(ns):
   delay=0 if n.endswith('_abd') else delays[n.split('_')[0]]
   for k in range(len(ts)-1):
    out[:,off+j]+=smooth((t-ts[k]-delay)/(ts[k+1]-ts[k]))*(values[k+1,j]-values[k,j])
  timeline.append(dict(part=side+'_fingers',keyframes=[dict(t_s=float(ts[k]),shape=key[1],amount=key[2]) for k,key in enumerate(keys)],flexion_stagger_s=delays))
 for side in {x[0] for x in episodes}:
  off=14 if side=='left' else 30;out[[0,-1],off:off+16]=d.rest[off:off+16]
 return out,timeline


def apply(d,i,t,q):
 return apply_phrases(d,t,q,EPISODES[i])

def parse_phrases(items,duration):
 episodes=[];ends={}
 for item in items:
  side=item['side'];start=float(item.get('start_s',0.))
  if side not in ('left','right') or not np.isfinite(start) or start<0:raise ValueError('Invalid hand phrase side/start')
  keys=[(float(k['t_s']),k['shape'],float(k.get('amount',1.))) for k in item['keys']]
  if len(keys)<2 or keys[0][0]!=0 or keys[0][1]!='rest' or keys[-1][1]!='rest':raise ValueError('Each hand phrase starts and ends in rest')
  times=np.array([k[0] for k in keys])
  if not np.isfinite(times).all() or np.any(np.diff(times)<=0):raise ValueError('Increasing finite hand phrase times required')
  for _,shape,amount in keys:
   if shape not in ('rest','open','curl','soft','broad','relaxed') or not np.isfinite(amount) or not 0<=amount<=2:raise ValueError('Invalid hand shape/amount')
  end=start+times[-1]+.12
  if end>duration or start<ends.get(side,-1):raise ValueError('Overlapping same-hand phrases or insufficient endpoint settling time')
  ends[side]=end;episodes.append((side,start,keys))
 return episodes
