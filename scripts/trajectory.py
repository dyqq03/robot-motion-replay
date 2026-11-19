"""C2 pass/stop waypoints and clock changes with explicitly protected static holds."""
import copy
import numpy as np
from scipy.interpolate import BPoly, CubicSpline, make_interp_spline


def interpolate(frames, values, times):
    """Legacy splines stay unchanged; explicit kind/hold_s opts into phase control."""
    values = np.asarray(values, float)
    knots = np.asarray([f['t'] for f in frames], float)
    if not any('kind' in f or 'hold_s' in f for f in frames):
        zero = np.zeros(values.shape[1])
        return make_interp_spline(knots, values, k=5,
            bc_type=([(1, zero), (2, zero)], [(1, zero), (2, zero)]))(times)
    kt, q, stops = [], [], []
    for i, (f, value) in enumerate(zip(frames, values)):
        kind = f.get('kind', 'pass')
        hold = float(f.get('hold_s', 0))
        if kind not in ('pass', 'stop') or not np.isfinite(hold) or hold < 0:
            raise ValueError('Use kind=pass/stop and finite hold_s >= 0')
        if hold and (i == len(frames)-1 or f['t'] + hold >= knots[i+1]):
            raise ValueError('A hold must end strictly before the next waypoint')
        kt.append(float(f['t'])); q.append(value)
        stops.append(kind == 'stop' or hold > 0 or i in (0, len(frames)-1))
        if hold:
            kt.append(f['t']+hold); q.append(value); stops.append(True)
    curve = phase_curve(np.array(kt), np.array(q), stops)
    return curve(times)


def phase_curve(knots, values, stops):
    """Shared derivatives make pass points C2; holds and endpoints have v=a=0."""
    seed = CubicSpline(knots, values, axis=0, bc_type='clamped')
    v, a = seed(knots, 1), seed(knots, 2)
    v[np.asarray(stops, bool)] = 0
    a[np.asarray(stops, bool)] = 0
    return BPoly.from_derivatives(knots, [[q, dq, ddq] for q, dq, ddq in zip(values, v, a)])


def retime_clock(t, q, speed_factor=1., protected_holds=()):
    """Keep poses and static hold durations; rescale all remaining time intervals.

    Piecewise affine joins occur only inside verified static intervals. No event
    stream is inferred or rewritten here. Output timestamps may be nonuniform.
    """
    t, q = np.asarray(t, float), np.asarray(q, float)
    factor = float(speed_factor)
    if not np.isfinite(factor) or factor <= 0:
        raise ValueError('speed_factor must be positive and finite')
    if len(t) < 3 or q.shape[0] != len(t) or t[0] != 0 or np.any(np.diff(t) <= 0):
        raise ValueError('Increasing synchronized clock starting at zero required')
    if not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('Finite clock and poses required')
    holds = sorted([list(map(float, h)) for h in protected_holds])
    last = -1.
    for start, end in holds:
        if not np.isfinite([start, end]).all() or start < 0 or end > t[-1] or end <= start or start < last:
            raise ValueError('Protected holds must be nonoverlapping intervals inside the clock')
        # Check interpolated boundaries AND every command segment intersecting
        # the hold: a nearby moving sample must not be hidden at an edge.
        lo = max(0, np.searchsorted(t, start, side='right')-1)
        hi = min(len(t), np.searchsorted(t, end, side='left')+1)
        if hi-lo < 2 or np.max(np.abs(q[lo:hi]-q[lo])) > 1e-8:
            raise ValueError('Protected hold contains motion; choose static boundaries')
        last = end
    def transform(x):
        x = np.asarray(x, float)
        held = sum((np.clip(x-start, 0, end-start) for start, end in holds), np.zeros_like(x))
        return (x-held)/factor + held
    # Preserve exact interval boundaries even when they fall between samples.
    boundaries = [v for h in holds for v in h]
    source_t = np.unique(np.r_[t, boundaries])
    new_q = np.column_stack([np.interp(source_t, t, q[:,j]) for j in range(q.shape[1])])
    return transform(source_t), new_q, transform


def map_timeline(episodes, transform):
    result = copy.deepcopy(episodes)
    for episode in result:
        source_times = [frame['t_s'] for frame in episode.get('keyframes', [])]
        for frame in episode.get('keyframes', []):
            frame['t_s'] = float(transform(frame['t_s']))
        # Delays are joint-specific and cannot be uniformly rescaled when a
        # protected interval splits a phrase; retain the source for provenance.
        if 'flexion_stagger_s' in episode or 'exported_flexion_keyframe_times_s' in episode:
            if 'exported_flexion_keyframe_times_s' in episode:
                mapped={finger:[float(transform(ts)) for ts in times] for finger,times in episode['exported_flexion_keyframe_times_s'].items()}
            else:
                delays=episode['flexion_stagger_s']
                mapped={finger:[float(transform(ts+delay)) for ts in source_times] for finger,delay in delays.items()}
            episode['exported_flexion_keyframe_times_s']=mapped
            spans={finger:np.array(times)-np.array([f['t_s'] for f in episode['keyframes']]) for finger,times in mapped.items()}
            if all(np.ptp(v)<1e-9 for v in spans.values()):
                episode['flexion_stagger_s']={finger:float(v[0]) for finger,v in spans.items()}
            else:
                if 'flexion_stagger_s' in episode:episode['source_flexion_stagger_s']=episode.pop('flexion_stagger_s')
                episode['exported_clock_note']='Nonuniform clock: use per-finger mapped keyframe times and command timestamps.'
    return result
