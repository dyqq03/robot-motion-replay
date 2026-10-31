"""Reusable, offline self-collision screening and adaptive trajectory validation.

Quick screening is cheap and conservative. Candidate checks query near convex
meshes, caching pairs whose relative joint coordinates did not change. Final
checks bound inter-sample movement; unresolved intervals never count as passes.
The caller's model flags, state, and generated commands are never modified.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RULES = ROOT / "configs/collision_rules.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _finite_number(value, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Collision rule must be a finite number")
    if value < 0 or (positive and value == 0):
        raise ValueError("Collision rule must be positive" if positive else "Negative collision rule")
    return float(value)


def load_rules(path=DEFAULT_RULES):
    rules = json.loads(Path(path).read_text())
    if rules.get("schema_version") != 1:
        raise ValueError("Unsupported collision rules schema")
    for key in ("ignore_same_welded_body", "ignore_direct_joint_neighbors"):
        if not isinstance(rules.get(key), bool):
            raise ValueError("Expected boolean collision topology rule: " + key)
    group = rules.get("collision_geom_group")
    if isinstance(group, bool) or not isinstance(group, int) or not 0 <= group < 6:
        raise ValueError("Invalid collision geom group")
    for key in ("default", "same_hand"):
        _finite_number(rules["clearance_m"][key])
    solver = rules["mesh_solver"]
    if solver.get("native_ccd") is not True:
        raise ValueError("Use native CCD: the legacy mesh distance solver is cutoff-sensitive")
    for key in ("tolerance", "distance_padding_m", "numeric_guard_m", "consistency_tolerance_m"):
        _finite_number(solver[key], positive=True)
    if solver["distance_padding_m"] <= solver["numeric_guard_m"]:
        raise ValueError("Distance padding must exceed the numeric guard")
    for section, key in ((solver, "iterations"), (rules["trajectory"], "maximum_depth"),
                         (rules["trajectory"], "maximum_pose_checks")):
        if isinstance(section[key], bool) or not isinstance(section[key], int) or section[key] < 1:
            raise ValueError("Invalid collision iteration budget: " + key)
    for key in ("coarse_hz", "minimum_interval_s", "maximum_wall_time_s"):
        _finite_number(rules["trajectory"][key], positive=True)
    seen = set()
    for field in ("excluded_pairs", "pair_overrides"):
        for row in rules[field]:
            names = row.get("geoms", [])
            if (not isinstance(names, list) or len(names) != 2 or names[0] == names[1]
                    or any(not isinstance(n, str) or not n for n in names)
                    or not isinstance(row.get("reason"), str) or not row["reason"].strip()):
                raise ValueError("A collision pair rule needs two exact geom names and a reason")
            pair = tuple(sorted(names))
            if pair in seen:
                raise ValueError("Duplicate or conflicting collision pair rule: " + str(pair))
            seen.add(pair)
            if field == "pair_overrides":
                _finite_number(row["clearance_m"])
    return rules


@dataclass(frozen=True)
class Pair:
    a: int
    b: int
    margin: float
    category: str
    dofs: tuple[int, ...]
    weights: tuple[float, ...]


class CollisionCheckError(ValueError):
    def __init__(self, report):
        self.report = report
        finding = (report.get("findings") or [{}])[0]
        super().__init__("Collision review required: " + report["status"] + "; " +
                         json.dumps(finding, ensure_ascii=False))


def require_pass(report):
    """Reject a draft export; the calling design loop can adjust and retry."""
    if (report.get("stage") != "trajectory" or report.get("status") != "passed"
            or report.get("can_export") is not True or report.get("certified_fraction") != 1.0):
        raise CollisionCheckError(report)
    return report


class CollisionChecker:
    def __init__(self, model, rules_path=DEFAULT_RULES):
        self.rules_path = Path(rules_path)
        self.rules = load_rules(self.rules_path)
        self.rules_sha256 = digest(self.rules_path)
        self.compound = None
        self.proxy_inflation = 0.
        if self.rules.get('proxy_manifest'):
            from collision_proxy import CompoundGeometry
            self.compound = CompoundGeometry(model, self.rules)
            self.proxy_inflation = .0001  # circumscribes thin component boxes
        # Native CCD and tolerances are isolated from the shadow/dynamics model.
        self.model = copy.copy(model)
        self.data = mujoco.MjData(self.model)
        self.model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_NATIVECCD)
        self.model.opt.ccd_iterations = self.rules["mesh_solver"]["iterations"]
        self.model.opt.ccd_tolerance = self.rules["mesh_solver"]["tolerance"]
        self.geom_ids = np.flatnonzero(model.geom_group == self.rules["collision_geom_group"])
        if not len(self.geom_ids):
            raise ValueError("No collision geometry in the configured group")
        # Bounded convex geometry; planes/heightfields need a separate scene policy.
        supported = {int(mujoco.mjtGeom.mjGEOM_MESH), int(mujoco.mjtGeom.mjGEOM_SPHERE),
                     int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_CAPSULE),
                     int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)}
        if any(int(model.geom_type[g]) not in supported for g in self.geom_ids):
            raise ValueError("Unsupported collision shape in self-collision group")
        if any(int(t) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
               for t in model.jnt_type) or model.nq != model.nv:
            raise ValueError("Collision path bounds require scalar hinge/slide joints")
        if np.any(model.body_jntnum > 1):
            raise ValueError("Collision path bounds require at most one joint per body")
        self.names = {int(g): model.geom(int(g)).name for g in self.geom_ids}
        if any(n is None for n in self.names.values()):
            raise ValueError("Name each collision geom")
        named = {v: k for k, v in self.names.items()}
        excluded, overrides = {}, {}
        for field, target in (("excluded_pairs", excluded), ("pair_overrides", overrides)):
            for row in self.rules[field]:
                if any(n not in named for n in row["geoms"]):
                    raise ValueError("Unknown collision geom in rule: " + str(row["geoms"]))
                target[tuple(sorted(named[n] for n in row["geoms"]))] = row
        chains = {int(g): self._joint_radii(int(g)) for g in self.geom_ids}
        self.exclusions = []
        self.pairs = []
        for i, a in enumerate(self.geom_ids):
            for b in self.geom_ids[i + 1:]:
                a, b = int(a), int(b)
                wa = int(model.body_weldid[model.geom_bodyid[a]])
                wb = int(model.body_weldid[model.geom_bodyid[b]])
                reason = None
                if self.rules["ignore_same_welded_body"] and wa == wb:
                    reason = "same welded body"
                elif self.rules["ignore_direct_joint_neighbors"] and (
                        int(model.body_weldid[model.body_parentid[wa]]) == wb or
                        int(model.body_weldid[model.body_parentid[wb]]) == wa):
                    reason = "direct joint neighbors after collapsing fixed links"
                if (a, b) in excluded:
                    reason = excluded[a, b]["reason"]
                if reason:
                    self.exclusions.append(dict(geoms=[self.names[a], self.names[b]], reason=reason))
                    continue
                side = lambda n: re.match(r"^(Left|Right)(Palm|Thumb|Index|Middle|Ring|Pinky)", n)
                left, right = side(self.names[a]), side(self.names[b])
                category = "same_hand" if left and right and left[1] == right[1] else "default"
                margin = float(self.rules["clearance_m"][category])
                if (a, b) in overrides:
                    margin = float(overrides[a, b]["clearance_m"])
                # Common ancestors move both shapes rigidly; only the exclusive
                # paths change their relative distance.
                joints = sorted(set(chains[a]) ^ set(chains[b]))
                dofs = tuple(int(model.jnt_qposadr[j]) for j in joints)
                weights = tuple(float(chains[a].get(j, chains[b].get(j))) for j in joints)
                self.pairs.append(Pair(a, b, margin, category, dofs, weights))
        if not self.pairs:
            raise ValueError("Collision rules exclude every pair")
        used = set(excluded) | set(overrides)
        if any(pair not in {(p.a, p.b) for p in self.pairs} and pair not in excluded for pair in used):
            raise ValueError("Pair override targets an automatically excluded assembly pair")
        self.a = np.array([p.a for p in self.pairs], dtype=int)
        self.b = np.array([p.b for p in self.pairs], dtype=int)
        self.margins = np.array([p.margin for p in self.pairs])
        self.motion_weights = np.zeros((len(self.pairs), model.nq))
        for i, p in enumerate(self.pairs):
            self.motion_weights[i, list(p.dofs)] = p.weights
        self._cache = {}
        self.queries = self.cache_hits = 0
        self._bounds = None

    def _joint_radii(self, geom):
        """Conservative reach from each ancestor joint to every shape point."""
        m = self.model
        radius = (np.linalg.norm(m.geom_pos[geom]) + np.linalg.norm(m.geom_aabb[geom, :3])
                  + np.linalg.norm(m.geom_aabb[geom, 3:]) + self.proxy_inflation)
        body = int(m.geom_bodyid[geom])
        result = {}
        while body:
            if m.body_jntnum[body]:
                j = int(m.body_jntadr[body])
                if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
                    result[j] = 1.
                    if not m.jnt_limited[j]:
                        raise ValueError("A sliding ancestor needs finite limits for reach bounds")
                    radius += float(np.max(np.abs(m.jnt_range[j] - m.qpos0[m.jnt_qposadr[j]])))
                else:
                    pivot = float(np.linalg.norm(m.jnt_pos[j]))
                    result[j] = radius + pivot
                    # Rotation about an off-origin pivot can move a point up to
                    # twice the pivot offset farther from this body's origin.
                    # Carry that reach to more distant ancestor joints too.
                    radius += 2. * pivot
            radius += float(np.linalg.norm(m.body_pos[body]))
            body = int(m.body_parentid[body])
        return result

    def _validate_qpos(self, qpos):
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.model.nq,) or not np.isfinite(qpos).all():
            raise ValueError("Expected a finite, complete model qpos vector in radians")
        for j in range(self.model.njnt):
            if not self.model.jnt_limited[j]:
                continue
            value = qpos[self.model.jnt_qposadr[j]]
            low, high = self.model.jnt_range[j]
            if value < low - 1e-5 or value > high + 1e-5:
                raise ValueError("Joint outside collision model limits: " + self.model.joint(j).name)
        return qpos

    def set_pose(self, qpos):
        self.data.qpos[:] = self._validate_qpos(qpos)
        mujoco.mj_forward(self.model, self.data)
        return self.data

    def _lower_bounds(self, data):
        m = self.model
        rot = data.geom_xmat.reshape(-1, 3, 3)
        center = data.geom_xpos + np.einsum("nij,nj->ni", rot, m.geom_aabb[:, :3])
        half = np.einsum("nij,nj->ni", np.abs(rot), m.geom_aabb[:, 3:])
        gap = np.maximum(np.abs(center[self.a] - center[self.b]) - half[self.a] - half[self.b], 0.)
        box = np.linalg.norm(gap, axis=1)
        sphere = (np.linalg.norm(data.geom_xpos[self.a] - data.geom_xpos[self.b], axis=1)
                  - m.geom_rbound[self.a] - m.geom_rbound[self.b])
        return np.maximum(box, sphere) - 2*self.proxy_inflation

    def quick(self, data):
        """Read fresh mj_forward data; potential proximity is not a collision."""
        self._validate_qpos(data.qpos)
        lower = self._lower_bounds(data)
        near = lower < self.margins + self.rules["mesh_solver"]["numeric_guard_m"]
        return dict(stage="quick", status="needs_mesh" if near.any() else "clear_bounds",
                    pair_count=len(self.pairs), near_pair_count=int(near.sum()),
                    risk_score=float(np.square(np.maximum(self.margins - lower, 0.)).sum()),
                    can_export=False, conservative_bounds_only=True)

    def candidate(self, data):
        """Check one selected candidate; return findings so design can continue."""
        result, _ = self._evaluate(data)
        return result

    def _evaluate(self, data, extra_distance=0.):
        self._validate_qpos(data.qpos)
        solver = self.rules["mesh_solver"]
        guard = solver["numeric_guard_m"]
        lower = self._lower_bounds(data)
        indices = np.flatnonzero(lower <= self.margins + guard + extra_distance)
        findings = []
        numerical = []
        before, hits = self.queries, self.cache_hits
        if self.compound is not None:
            self.compound.forward(data.qpos)
        for i in indices:
            pair = self.pairs[i]
            key = tuple(float(data.qpos[d]) for d in pair.dofs)
            cached = self._cache.get(int(i))
            if cached is not None and cached[0] == key:
                distance, inconsistent = cached[1:]
                self.cache_hits += 1
            else:
                cutoff = pair.margin + solver["distance_padding_m"]
                if self.compound is not None:
                    distance, inconsistent, count = self.compound.distances(pair.a, pair.b, cutoff, guard)
                    self.queries += count
                else:
                    distance = float(mujoco.mj_geomDistance(self.model, data, pair.a, pair.b, cutoff, None))
                    self.queries += 1
                    # A second nearby cutoff detects the known cutoff dependence of
                    # older MuJoCo mesh solvers. Never turn disagreement into a pass.
                    second = float(mujoco.mj_geomDistance(self.model, data, pair.a, pair.b, cutoff * 1.5, None))
                    self.queries += 1
                    inconsistent = (not math.isfinite(distance) or not math.isfinite(second) or
                        (distance < cutoff - guard and abs(distance - second) > solver["consistency_tolerance_m"]))
                    distance = min(distance, second)
                self._cache[int(i)] = (key, distance, inconsistent)
            names = [self.names[pair.a], self.names[pair.b]]
            if inconsistent:
                numerical.append(dict(geoms=names, reason="mesh distance solver disagreement"))
                lower[i] = -math.inf
            else:
                # Bounds are conservative; a smaller solver result may be kept
                # conservatively even when a native solver is slightly noisy.
                lower[i] = distance - guard
                if lower[i] < pair.margin:
                    findings.append(dict(geoms=names, distance_m=distance,
                        required_clearance_m=pair.margin, category=pair.category,
                        kind="mesh_overlap" if distance < -guard else "clearance"))
        findings.sort(key=lambda f: f["distance_m"] - f["required_clearance_m"])
        status = "failed" if findings else "inconclusive" if numerical else "passed"
        result = dict(stage="candidate", status=status, pair_count=len(self.pairs),
            near_pair_count=len(indices), mesh_queries=self.queries - before,
            cached_pairs=self.cache_hits - hits, findings=findings, numerical_findings=numerical,
            minimum_clearance_lower_bound_m=float(lower.min()) if np.isfinite(lower).all() else None,
            can_export=False)
        return result, lower

    def check_trajectory(self, times, qpos):
        """Validate the entire piecewise-linear qpos path with movement bounds.

        At 10 Hz anchors, safe intervals are certified from distance/reach
        bounds. Only uncertain intervals are bisected. Original intermediate
        commands contribute total variation, so a loop between two equal
        anchors cannot be silently skipped. Resource limits are inconclusive.
        """
        times, qpos = np.asarray(times, dtype=float), np.asarray(qpos, dtype=float)
        if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
                or np.any(np.diff(times) <= 0) or qpos.shape != (len(times), self.model.nq)
                or not np.isfinite(qpos).all()):
            raise ValueError("Invalid synchronized collision trajectory")
        for row in qpos:
            self._validate_qpos(row)
        settings = self.rules["trajectory"]
        start = time.perf_counter()
        queries, hits = self.queries, self.cache_hits
        variation = np.vstack([np.zeros(self.model.nq), np.cumsum(np.abs(np.diff(qpos, axis=0)), axis=0)])
        samples = {}
        findings, uncertain = [], []
        certified = 0.
        intervals = 0
        def interpolate(seconds, array):
            k = min(int(np.searchsorted(times, seconds, side="right")) - 1, len(times) - 2)
            k = max(k, 0)
            u = (seconds - times[k]) / (times[k + 1] - times[k])
            return array[k] + u * (array[k + 1] - array[k])
        def node(seconds):
            key = float(seconds)
            if key not in samples:
                if (len(samples) >= settings["maximum_pose_checks"] or
                        time.perf_counter() - start >= settings["maximum_wall_time_s"]):
                    return None
                self.set_pose(interpolate(seconds, qpos))
                result, lower = self._evaluate(self.data, self.rules['mesh_solver']['distance_padding_m'])
                result["time_s"] = key
                samples[key] = (result, lower)
            return samples[key]
        def interval(left, right, depth):
            nonlocal certified, intervals
            intervals += 1
            a, b = node(left), node(right)
            if a is None or b is None:
                uncertain.append(dict(start_s=left, end_s=right, reason="checking budget exhausted"))
                return
            for entry in (a, b):
                result, _ = entry
                if result["status"] != "passed":
                    findings.extend(dict(f, time_s=result["time_s"]) for f in result["findings"])
                    if result["numerical_findings"]:
                        uncertain.append(dict(start_s=left, end_s=right,
                                              reason="mesh distance solver disagreement"))
                    return
            movement = np.einsum("ij,j->i", self.motion_weights,
                                 interpolate(right, variation) - interpolate(left, variation))
            # Either endpoint can certify the complete intervening path.
            separation = np.maximum(a[1], b[1]) - movement
            if np.all(separation >= self.margins + self.rules["mesh_solver"]["numeric_guard_m"]):
                certified += right - left
                return
            if depth >= settings["maximum_depth"] or right - left <= settings["minimum_interval_s"]:
                uncertain.append(dict(start_s=left, end_s=right,
                    reason="clearance between samples could not be certified"))
                return
            midpoint = (left + right) / 2.
            interval(left, midpoint, depth + 1)
            if not findings:
                interval(midpoint, right, depth + 1)
        # Check baseline first; fail quickly with useful diagnostics for the next design trial.
        first = node(float(times[0]))
        if first is not None and first[0]["status"] != "passed":
            findings.extend(dict(f, time_s=float(times[0])) for f in first[0]["findings"])
            if first[0]["numerical_findings"]:
                uncertain.append(dict(start_s=float(times[0]), end_s=float(times[0]), reason="baseline numerical uncertainty"))
        else:
            anchors = np.r_[np.arange(times[0], times[-1], 1. / settings["coarse_hz"]), times[-1]]
            for left, right in zip(anchors[:-1], anchors[1:]):
                interval(float(left), float(right), 0)
                if findings:
                    break
        unique = {(tuple(f["geoms"]), f["time_s"]): f for f in findings}
        findings = sorted(unique.values(), key=lambda f: (f["time_s"], f["distance_m"]))
        duration = float(times[-1] - times[0])
        complete = abs(certified - duration) < max(1e-9, duration * 1e-10)
        status = "failed" if findings else "passed" if complete and not uncertain else "inconclusive"
        return dict(schema_version=1, stage="trajectory", status=status, can_export=status == "passed",
            scope="convex robot self-collision; configured group only; excluded assembly pairs listed",
            interpolation="piecewise linear model qpos between all input commands",
            rules_sha256=self.rules_sha256, rules=self.rules, mujoco_version=mujoco.__version__,
            input_frames=len(times), duration_s=duration, checked_poses=len(samples), interval_visits=intervals,
            certified_duration_s=certified, certified_fraction=1. if status == "passed" else min(1., certified / duration),
            mesh_queries=self.queries - queries, cached_pair_queries=self.cache_hits - hits,
            elapsed_s=time.perf_counter() - start, pair_count=len(self.pairs),
            excluded_pairs=self.exclusions, findings=findings[:100], unresolved_intervals=uncertain[:100],
            findings_truncated=len(findings)>100, unresolved_intervals_truncated=len(uncertain)>100,
            hardware_execution_validated=False, physical_contact_response_enabled=False)
