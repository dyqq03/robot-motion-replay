"""Offline behavioral regression for direction/limit audits, neck return and hand phrases."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import copy
import json
import tempfile
import time
from types import SimpleNamespace as NS
from pathlib import Path

import h5py
import numpy as np
from common import WORK, read, write, refresh_manifest
import motion
import alignment_audit
from calibrated_hand_pose import CalibratedHandPose
from conversational_hands import parse_phrases
from hand_limits import validate_joint_degrees
from offline_backend import player_class, Sink, validate as backend


def rejects(fn):
    try:
        fn()
    except ValueError:
        return
    raise AssertionError('Expected invalid input to be rejected')


def run(full=False):
    checks = []
    audit = alignment_audit.run()
    assert len(audit['joints']) == 50
    assert not audit['live_driver_directions_verified']
    assert audit['physical_alignment_status'] != 'passed'
    checks.append('numerical_alignment_is_distinct_from_physical_verification')
    cal = CalibratedHandPose()
    for side in ['left', 'right']:
        model = cal.models[side + '_hand']
        values = np.mean(model.rom, axis=1)[None, :]
        result = validate_joint_degrees(cal, side, values)
        assert result['encoder_roundtrip_error_deg'] < 1e-9
        bad = values.copy()
        bad[0, 0] = model.rom[0][1] + .1
        rejects(lambda: validate_joint_degrees(cal, side, bad))
    checks.append('calibrated_rom_encoder_roundtrip_and_out_of_range_rejection')
    P = player_class(WORK)
    player = P.__new__(P)
    player._fbp = dict(chains=['neck'])
    player._neck_names = ['neck_joint1', 'neck_joint2']
    current = [.06, -.04]
    player._fb = NS(_neck_received=time.monotonic(), neck_pos=lambda names: current)
    player._pub_neck_traj = Sink()
    player._log = NS(info=lambda message: None)
    player._begin_home_return()
    message = player._pub_neck_traj.messages[-1]
    positions = np.array([p.positions for p in message.points])
    np.testing.assert_allclose(positions[0], current, atol=1e-12)
    np.testing.assert_allclose(positions[-1], [0, 0], atol=1e-12)
    assert player._fbp['home_duration'] == 1.
    assert np.all(np.diff(positions[:, 0]) <= 0) and np.all(np.diff(positions[:, 1]) >= 0)
    player._fb._neck_received = 0.
    rejects(player._begin_home_return)
    checks.append('actual_player_neck_returns_smoothly_to_zero_and_rejects_stale_feedback')
    with tempfile.TemporaryDirectory(prefix='alignment_hands_', dir=WORK) as folder:
        tmp = Path(folder)
        phrase = dict(side='right', start_s=.2, keys=[dict(t_s=0, shape='rest'),
                    dict(t_s=1.3, shape='open', amount=.08),
                    dict(t_s=3.1, shape='soft', amount=.08), dict(t_s=5.3, shape='rest')])
        spec = dict(package='fb_test_hand_phrase_v1', name='离线手型短语测试', duration_s=6, hand_phrases=[phrase])
        write(tmp / 'design.json', spec)
        draft = motion.create(tmp / 'design.json', tmp / spec['package'])
        motion.kinematics(draft)
        refresh_manifest(draft)
        backend(draft)
        meta = read(draft / 'motion.json')
        with h5py.File(draft / 'fullbody.hdf5') as h:
            for key in ['arms/cmd_left', 'arms/cmd_right', 'neck/q', 'hand_left/q', 'eyou/q']:
                assert len(h[key]) == 0
            q = h['hand_right/q'][:]
            np.testing.assert_allclose(q[0], q[-1], atol=1e-12)
            j = meta['hand_joint_names']['right'].index('index_mcp')
            assert q[:, j].min() < q[0, j] - .1 and q[:, j].max() > q[0, j] + .1
        for bad_phrases in [[dict(phrase, side='wrong')], [dict(phrase, start_s=5)], [phrase, phrase]]:
            rejects(lambda: parse_phrases(bad_phrases, 6))
        checks.append('open_then_curl_single_hand_stream_and_invalid_timing_rejection')
        scaled = copy.deepcopy(spec)
        scaled['package'] = 'fb_test_hand_phrase_slow_v1'
        scaled['speed_caps_deg_s'] = dict(hand_right=.05)
        write(tmp / 'scaled.json', scaled)
        other = motion.create(tmp / 'scaled.json', tmp / scaled['package'])
        m = read(other / 'motion.json')
        scale = m['automatic_duration_factor']
        assert scale > 1
        np.testing.assert_allclose(m['semantic_hand_timeline'][0]['keyframes'][1]['t_s'], 1.5 * scale)
        np.testing.assert_allclose(m['semantic_hand_timeline'][0]['flexion_stagger_s']['pinky'], .12 * scale)
        checks.append('speed_extension_scales_semantic_timeline_and_finger_delays')
        if full:
            motion.check(draft)
            motion.render(draft, fps=12)
            checks.append('new_hand_phrase_complete_collision_path_and_full_video')
    return dict(status='passed', full=full, checks=checks, network_access=False, robot_commands_sent=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    result = run(args.full)
    write(WORK / 'verification/alignment_hands_test.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
