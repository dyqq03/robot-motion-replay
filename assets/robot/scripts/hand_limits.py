"""Read-only calibrated joint/encoder range checks for the active hardware YAMLs."""
import copy
import numpy as np

def validate_joint_degrees(calibration,side,values):
 model=copy.deepcopy(calibration.models[side+'_hand']);model.wrap=[0.]*16
 values=np.asarray(values,float)
 if values.ndim!=2 or values.shape[1]!=16 or not np.isfinite(values).all():raise ValueError('Expected finite N x 16 hand joint degrees')
 rom=np.asarray(model.rom);limits=np.asarray(model.limits)
 if np.any(values<rom[:,0]-1e-7) or np.any(values>rom[:,1]+1e-7):raise ValueError('Hand calibrated ROM exceeded: '+side)
 maximum=0.
 for row in values:
  motor=np.asarray(model.to_motor(row.tolist()))
  if np.any(motor<limits[:,0]-1e-9) or np.any(motor>limits[:,1]+1e-9):raise ValueError('Hand encoder range exceeded: '+side)
  maximum=max(maximum,float(np.max(np.abs(np.asarray(model.to_joint(motor.tolist()))-row))))
 if maximum>1e-9:raise ValueError('Hand encoder/joint roundtrip mismatch: '+side)
 return dict(joint_rom=True,motor_limits=True,encoder_roundtrip_error_deg=maximum)
