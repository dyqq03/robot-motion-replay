"""Conservative triangle partition: clip every face at spatial split planes.

Leaf hulls cover every original surface triangle, including open meshes. No
geometry is discarded. New edges lie on the source triangle or split plane.
"""
import numpy as np


def clip(poly,axis,plane,positive):
    output=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da,db=a[axis]-plane,b[axis]-plane
        ina=da>=0 if positive else da<=0
        inb=db>=0 if positive else db<=0
        if ina:output.append(a)
        if ina != inb:
            output.append(a+(b-a)*(da/(da-db)))
    return [np.array([output[0],output[k],output[k+1]]) for k in range(1,len(output)-1)]


def partition(triangles,extent=.0125):
    lo=triangles.min(axis=(0,1));hi=triangles.max(axis=(0,1))
    axis=int(np.argmax(hi-lo))
    if hi[axis]-lo[axis]<=extent:
        return [triangles.reshape(-1,3)]
    plane=(hi[axis]+lo[axis])*.5
    positions=triangles[:,:,axis]
    negative=np.max(positions,axis=1)<=plane
    positive=np.min(positions,axis=1)>=plane
    cross=~(negative|positive)
    left=list(triangles[negative]);right=list(triangles[positive&~negative])
    for triangle in triangles[cross]:
        left.extend(clip(triangle,axis,plane,False))
        right.extend(clip(triangle,axis,plane,True))
    result=[]
    for side in (left,right):
        if side:result.extend(partition(np.asarray(side),extent))
    return result
