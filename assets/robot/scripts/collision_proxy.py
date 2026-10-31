"""Optional compound convex geometry, isolated from the visual/command model."""
from __future__ import annotations
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial import cKDTree
from collision_checker import digest

ROOT=Path(__file__).resolve().parents[1]


def geometry_hashes(rules):
    value=rules.get("proxy_manifest")
    if not value:return {}
    path=ROOT/value
    manifest=json.loads(path.read_text())
    model=ROOT/manifest["model"]
    if digest(model)!=manifest["model_sha256"]:
        raise ValueError("Collision proxy model differs from manifest")
    source=ROOT/manifest["source_model"]
    if digest(source)!=manifest["source_model_sha256"]:
        raise ValueError("Collision proxy was built for another visual model")
    for group in manifest['groups']:
        if digest(ROOT/group['source_mesh'])!=group['source_sha256']:
            raise ValueError('Collision proxy source mesh changed')
    return {value:digest(path),manifest["model"]:digest(model)}


class CompoundGeometry:
    def __init__(self,model,rules):
        self.hashes=geometry_hashes(rules)
        self.tolerance=rules["mesh_solver"]["consistency_tolerance_m"]
        manifest=json.loads((ROOT/rules['proxy_manifest']).read_text())
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/manifest['model']))
        if ([model.joint(i).name for i in range(model.njnt)] !=
                [self.model.joint(i).name for i in range(self.model.njnt)]):
            raise ValueError('Collision proxy joint names/order changed')
        np.testing.assert_allclose(model.jnt_range,self.model.jnt_range,atol=1e-12)
        self.model.opt.enableflags|=int(mujoco.mjtEnableBit.mjENBL_NATIVECCD)
        self.model.opt.ccd_iterations=rules['mesh_solver']['iterations']
        self.model.opt.ccd_tolerance=rules['mesh_solver']['tolerance']
        self.data=mujoco.MjData(self.model)
        self.groups={model.geom(row['geom']).id:np.array([self.model.geom(n).id for n in row['parts']],dtype=int)
                     for row in manifest['groups']}
        required=set(np.flatnonzero(model.geom_group==rules['collision_geom_group']))
        if set(self.groups)!=required:raise ValueError('Proxy does not cover every collision geom')
        self.mounts={}
        mujoco.mj_forward(self.model,self.data)
        for mount in rules.get('compound_mounts',[]):
            if not mount.get('reason') or mount['radius_m']>.03:raise ValueError('Unreviewed compound mount')
            pair=tuple(model.geom(n).id for n in mount['geoms'])
            joint=self.model.joint(mount['joint']).id
            center=self.data.xanchor[joint]
            inside={}
            for original in pair:
                ids=self.groups[original];selected=[]
                for g in ids:
                    mesh=self.model.geom_dataid[g];start=self.model.mesh_vertadr[mesh]
                    vertices=self.model.mesh_vert[start:start+self.model.mesh_vertnum[mesh]]
                    xyz=vertices@self.data.geom_xmat[g].reshape(3,3).T+self.data.geom_xpos[g]
                    if np.linalg.norm(xyz-center,axis=1).max()<=mount['radius_m']:
                        selected.append(g)
                inside[original]=np.array(selected,dtype=int)
            self.mounts[frozenset(pair)]=inside

    def forward(self,qpos):
        self.data.qpos[:]=qpos;mujoco.mj_forward(self.model,self.data)
        rot=self.data.geom_xmat.reshape(-1,3,3)
        self.center=self.data.geom_xpos+np.einsum('nij,nj->ni',rot,self.model.geom_aabb[:,:3])
        self.trees={}
        self.half=np.einsum('nij,nj->ni',abs(rot),self.model.geom_aabb[:,3:])

    def distances(self,a,b,cutoff,guard):
        ga,gb=self.groups[a],self.groups[b]
        if len(ga)*len(gb)>10000:
            if b not in self.trees:
                self.trees[b]=cKDTree(self.data.geom_xpos[gb])
            radius=self.model.geom_rbound[ga]+float(self.model.geom_rbound[gb].max())+cutoff
            lists=self.trees[b].query_ball_point(self.data.geom_xpos[ga],radius)
            count=np.array([len(row) for row in lists])
            if not count.any():return cutoff,False,0
            aa=np.repeat(ga,count);bb=gb[np.concatenate(lists).astype(int)]
        else:
            aa,bb=np.meshgrid(ga,gb,indexing='ij')
            aa,bb=aa.ravel(),bb.ravel()
        gaps=np.maximum(abs(self.center[aa]-self.center[bb])-self.half[aa]-self.half[bb],0.)
        lower=np.linalg.norm(gaps,axis=1)
        sphere=np.linalg.norm(self.data.geom_xpos[aa]-self.data.geom_xpos[bb],axis=1)-self.model.geom_rbound[aa]-self.model.geom_rbound[bb]
        lower=np.maximum(lower,sphere)
        mount=self.mounts.get(frozenset((a,b)))
        if mount:
            lower[np.isin(aa,mount[a])&np.isin(bb,mount[b])]=np.inf
        near=np.flatnonzero(lower<=cutoff)
        outside=lower>cutoff
        first=second=min(float(lower[outside].min()),cutoff) if outside.any() else cutoff
        queries=0
        inconsistent=False
        for k in near[np.argsort(lower[near])]:
            if lower[k]>=min(first,second):break
            x=float(mujoco.mj_geomDistance(self.model,self.data,int(aa[k]),int(bb[k]),cutoff,None))
            y=float(mujoco.mj_geomDistance(self.model,self.data,int(aa[k]),int(bb[k]),cutoff*1.5,None))
            if (not np.isfinite(x) or not np.isfinite(y) or
                (min(x,y)<cutoff-guard and abs(x-y)>self.tolerance)):
                inconsistent=True
            first=min(first,x);second=min(second,y);queries+=2
        return min(first,second),inconsistent,queries
