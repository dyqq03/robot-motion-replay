"""Exercise the bundled actual player via AST and message sinks; no ROS imports."""
import ast,__future__,json,math,time
from types import SimpleNamespace as NS
from pathlib import Path
import h5py,numpy as np
from common import ROBOT,digest,inputs

SOURCE=ROBOT/'backend/fullbody_player.py'
HELPER=ROBOT/'backend/convert_bag.py'
class Trajectory:
 def __init__(self):self.joint_names=[];self.points=[]
class Point:
 def __init__(self,**kw):self.__dict__.update(kw)
class Duration:
 def __init__(self,seconds):self.seconds=seconds
 def to_msg(self):return self.seconds
class Sink:
 def __init__(self):self.messages=[]
 def publish(self,message):self.messages.append(message)

def player_class(root):
 cls=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='FullbodyPlayer')
 nodes=[n for n in ast.parse(HELPER.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='stitch_gaps' or isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='GAP_THRESHOLD_S' for t in n.targets)]
 ns=dict(_h5py=h5py,json=json,FULLBODY_ROOT=Path(root),ARM_JOINT_NUMS=(1,2,3,4,5,7),JointTrajectory=Trajectory,JointTrajectoryPoint=Point,Duration=Duration,STALE_TOL=.02,_hands_disabled=lambda:False,math=math,time=time,JointState=lambda:NS(),Float64MultiArray=lambda **kw:NS(**kw))
 exec(compile(ast.Module(body=nodes+[cls],type_ignores=[]),str(SOURCE),'exec',flags=__future__.annotations.compiler_flag),ns)
 return ns['FullbodyPlayer']

def validate(directory):
 directory=Path(directory);P=player_class(directory.parent)
 p=P.__new__(P);p._fbp={};p._neck_names=['neck_joint1','neck_joint2'];p._snapshot=lambda:None;p._log=NS(info=lambda message:None)
 sinks={k:Sink() for k in ['hand','hand_left','hand_right','body']}
 p._pub_hand=sinks['hand'];p._pub_hand_left=sinks['hand_left'];p._pub_hand_right=sinks['hand_right'];p._pub_eyou_cmd=sinks['body']
 result=p._fbp_load(directory.name);assert result['ok'],result
 m=json.loads((directory/'motion.json').read_text())
 arms=bool(m.get('joint_names'));neck=bool(m.get('neck_joint_names'));hands=list(m.get('hand_joint_names',{}))
 expected=(['arms'] if arms else [])+(['neck'] if neck else [])+['hand_'+s for s in hands]
 assert result['chains']==expected,(result['chains'],expected)
 with h5py.File(directory/'fullbody.hdf5') as f:
  for direction,sign in [('fwd',1),('rev',-1)]:
   if arms:
    for side,suffix in [('left','l'),('right','r')]:
     msg=p._fbp['prebuilt']['arms'][direction][side];q=f['arms/cmd_'+side][:];ts=f['arms/t_arm_cmd_'+suffix][:]
     if sign<0:q=q[::-1];ts=ts[-1]-ts[::-1]
     assert msg.joint_names==[f'openarm_{side}_joint{j}' for j in [1,2,3,4,5,7]]
     np.testing.assert_allclose([x.positions for x in msg.points],q,rtol=0,atol=1e-12)
     np.testing.assert_allclose([x.time_from_start for x in msg.points],ts,rtol=0,atol=1e-12)
   if neck:
    msg=p._fbp['prebuilt']['neck'][direction];q=f['neck/q'][:];ts=f['neck/t'][:]
    if sign<0:q=q[::-1];ts=ts[-1]-ts[::-1]
    assert msg.joint_names==['neck_joint1','neck_joint2']
    np.testing.assert_allclose([x.positions for x in msg.points],q,rtol=0,atol=1e-12)
    np.testing.assert_allclose([x.time_from_start for x in msg.points],ts,rtol=0,atol=1e-12)
   for ts in np.linspace(0,p._fbp['T'],31)[::sign]:
    p._fbp['last_stream']=-100.;p._fbp_stream_tick(float(ts),sign)
    for side in hands:
     key='hand_'+side;msg=sinks[key].messages[-1];qs=f[key+'/q'][:];times=f[key+'/t'][:]
     np.testing.assert_allclose(msg.position,[np.interp(ts,times,qs[:,j]) for j in range(16)],rtol=0,atol=1e-10)
     assert msg.name==m['hand_joint_names'][side]
   assert not sinks['body'].messages and not sinks['hand'].messages
   for side in ['left','right']:
    if side not in hands:assert not sinks['hand_'+side].messages
 return dict(status='passed',backend_source='backend/fullbody_player.py',backend_sha256=digest(SOURCE),helper_sha256=digest(HELPER),input_sha256=inputs(directory),loaded_chains=result['chains'],forward_reverse_messages_checked=True,hand_stream_unit='calibrated_joint_deg',body_messages=0,ros_initialized=False,network_access=False,serial_access=False)

