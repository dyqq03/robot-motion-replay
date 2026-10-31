"""Build conservative component hulls without modifying the active shadow model.

Every source triangle belongs to exactly one connected component. Its component
hull contains every corner, hence the entire triangle. Unlike a single hull of a
whole mechanical assembly, separate hulls preserve spaces between disconnected
parts. Original visual assets, transforms, joints and calibration are untouched.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial import ConvexHull, QhullError
import trimesh

from collision_checker import digest
from proxy_partition import partition

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "generated/collision_proxy_doc50_20261021"


def component_hull(vertices):
    vertices = np.unique(vertices, axis=0)
    try:
        h = ConvexHull(vertices)
        if h.volume < 1e-14:
            raise QhullError("degenerate component")
        residual = np.max(vertices @ h.equations[:, :3].T + h.equations[:, 3])
        if residual > 1e-8:
            raise ValueError("Hull does not cover its source component")
        return vertices[h.vertices], float(residual), False
    except QhullError:
        # Keep even zero-volume triangles/fasteners as a conservative thin box.
        lo, hi = vertices.min(axis=0)-.00005, vertices.max(axis=0)+.00005
        return np.array([(x,y,z) for x in (lo[0],hi[0]) for y in (lo[1],hi[1])
                         for z in (lo[2],hi[2])]), 0., True


def build(output=OUT):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = ROOT / "generated/humanoid_shadow.xml"
    xml = ET.parse(source).getroot()
    assets = xml.find("asset")
    original_assets = {e.get("name"): (source.parent/e.get("file")).resolve()
                       for e in assets.findall("mesh")}
    geoms = [e for e in xml.iter("geom") if e.get("group") == "3"]
    for e in list(assets):
        if e.tag == "mesh": assets.remove(e)
    parents = {e: p for p in xml.iter() for e in p}
    for e in list(xml.iter("geom")):
        if e.get("group") != "3": parents[e].remove(e)
    rows = []
    for g in geoms:
        path = original_assets[g.get("mesh")]
        mesh = trimesh.load_mesh(path, process=True)
        # trimesh.split repairs holes and can add faces. Partition the source
        # adjacency directly so that isolated/open triangles are retained too.
        labels = trimesh.graph.connected_components(mesh.face_adjacency,
                    nodes=np.arange(len(mesh.faces)), min_len=1)
        if sum(len(index) for index in labels) != len(mesh.faces):
            raise ValueError("Component partition changed source faces: " + str(path))
        # Preserve internal concavities of torso and thumb sockets as well.
        refined = any(tag in g.get('name') for tag in ('WaistYaw','Palm__','ThumbPP','ShoulderRoll','ShoulderYaw','ElbowPitch','WristRoll','WristPitch'))
        components=[]
        for index in labels:
            triangles=mesh.vertices[mesh.faces[index]]
            extent=.006 if any(tag in g.get('name') for tag in ('Palm__','ThumbPP')) else .0125
            components.extend(partition(triangles,extent) if refined else [triangles.reshape(-1,3)])
        parent = parents[g]
        names = []
        coverage = []
        for i, component in enumerate(components):
            points, error, thin_box = component_hull(component)
            name = g.get("name") + f"__part_{i:03d}"
            meshname = name + "_mesh"
            ET.SubElement(assets, "mesh", name=meshname,
                vertex=" ".join(f"{v:.10g}" for v in points.ravel()))
            child = copy.deepcopy(g)
            child.set("name", name); child.set("mesh", meshname)
            parent.append(child); names.append(name)
            coverage.append(dict(clipped_triangle_vertices=len(component),
                hull_vertices=len(points),maximum_halfspace_residual_m=error,thin_box=thin_box))
        parent.remove(g)
        rows.append(dict(geom=g.get("name"),body=parent.get("name"),parts=names,
            source_mesh=str(path.relative_to(ROOT)),source_sha256=digest(path),
            method="connected_components_then_exact_triangle_plane_clipping" if refined else "connected_component_convex_hulls",coverage=coverage,
            all_source_triangles_covered=True))
        print(g.get("name"),len(components),"parts",flush=True)
    ET.indent(xml)
    target=output/"model.xml"
    ET.ElementTree(xml).write(target,encoding="utf-8",xml_declaration=True)
    original=mujoco.MjModel.from_xml_path(str(source))
    proxy=mujoco.MjModel.from_xml_path(str(target))
    if ([original.joint(i).name for i in range(original.njnt)] !=
            [proxy.joint(i).name for i in range(proxy.njnt)]):
        raise ValueError("Proxy changed joint coordinates")
    np.testing.assert_allclose(original.jnt_range,proxy.jnt_range,atol=1e-12)
    d,p=mujoco.MjData(original),mujoco.MjData(proxy)
    max_error=0.
    for f in (0.,.3,.7):
        d.qpos[:]=p.qpos[:]=(1-f)*original.jnt_range[:,0]+f*original.jnt_range[:,1]
        mujoco.mj_forward(original,d);mujoco.mj_forward(proxy,p)
        for j in range(original.nbody):
            name=original.body(j).name
            if name is None:continue
            k=proxy.body(name).id
            max_error=max(max_error,float(abs(d.xpos[j]-p.xpos[k]).max()),
                          float(abs(d.xmat[j]-p.xmat[k]).max()))
    if max_error>1e-10:raise ValueError("Proxy changed forward kinematics")
    report=dict(schema_version=1,source_model="generated/humanoid_shadow.xml",
        source_model_sha256=digest(source),model=str(target.relative_to(ROOT)),model_sha256=digest(target),
        builder_sha256=digest(Path(__file__)),source_geoms=len(rows),proxy_geoms=proxy.ngeom,
        joint_coordinates_unchanged=True,maximum_fk_error=max_error,
        original_visual_model_unchanged=digest(source),groups=rows,
        coverage="Every source triangle is partitioned by exact plane clipping; each fragment is contained in its leaf convex hull. No source face or connected component omitted.",
        hardware_geometry_verified=False)
    (output/"manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print('Completed',len(rows),'source geoms ->',proxy.ngeom,'convex parts',flush=True)
    return report


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=OUT)
    build(parser.parse_args().output)
