"""Read-only local coordinate, ROM and encoder audit; never certifies physical alignment."""
import argparse
import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')
import numpy as np
from common import SKILL, ROBOT, read, write, digest, reference_fingerprint
from model_snapshot import verify
from calibrated_hand_pose import CalibratedHandPose
from hand_limits import validate_joint_degrees
from motion_baseline import hand_baseline_deg


def run():
    snapshot = verify()
    cal = CalibratedHandPose()
    hand_reference = hand_baseline_deg()
    arm = read(ROBOT / 'configs/arm_motion_defaults.json')
    neck = read(ROBOT / 'configs/neck_motion_defaults.json')
    reference = dict(neck['neck'])
    for side, values in arm['arms'].items():
        reference.update({f'openarm_{side}_joint{j}': v for j, v in zip(arm['arm_joint_order'], values)})
    reference.update(hand_reference)
    mappings = {group: read(ROBOT / f'configs/{group}_mapping.json') for group in ['arm', 'neck', 'body', 'hand']}
    joints = []
    for group, mapping in mappings.items():
        for row in mapping['joints']:
            sign, offset = row['sign'], row['offset_rad']
            if sign not in (-1, 1) or not math.isfinite(offset):
                raise ValueError('Invalid model sign/offset: ' + row['source_joint'])
            lo, hi = row['real_limits_rad']
            if not math.isfinite(lo + hi) or lo >= hi:
                raise ValueError('Invalid logical bounds: ' + row['source_joint'])
            expected = sorted([sign * lo + offset, sign * hi + offset])
            np.testing.assert_allclose(expected, row['model_limits_rad'], rtol=0, atol=1e-8)
            name = row['source_joint']
            ref = reference.get(name)
            if ref is not None and not lo - 1e-7 <= math.radians(ref) <= hi + 1e-7:
                raise ValueError('Reference exceeds logical limits: ' + name)
            joints.append(dict(group=group, logical_joint=name, model_joint=row['model_joint'],
                               logical_limits_deg=np.rad2deg([lo, hi]).tolist(),
                               model_limits_deg=np.rad2deg(expected).tolist(), model_sign=sign,
                               model_offset_rad=offset, model_zero_logical_deg=math.degrees(-offset / sign),
                               logical_zero_model_rad=offset, motion_reference_deg=ref,
                               physical_zero_verified=bool(mapping.get('physical_zero_verified'))))
    hands = {}
    encoder_zero = cal.encoder_zero()
    for side in ['left', 'right']:
        model = cal.models[side + '_hand']
        values = [hand_reference[side + '/' + name] for name in model.names]
        checks = validate_joint_degrees(cal, side, np.array([[0.] * 16, values]))
        hands[side] = dict(serial=cal.settings['hands'][side]['serial'], checks=checks,
                           joints=[dict(name=name, motor_id=mid, motor_direction=-1 if inv else 1,
                                        logical_rom_deg=rom, encoder_limits_rad=limits,
                                        motor_rad_per_joint_deg=ratio,
                                        logical_zero_encoder_rad=zero,
                                        reference_deg=ref)
                                   for name, mid, inv, rom, limits, ratio, zero, ref in
                                   zip(model.names, model.ids, model.inverted, model.rom,
                                       model.limits, model.ratios, encoder_zero[side]['position'], values)])
    profile = read(SKILL / 'assets/server_profile.json')
    active = read(SKILL / 'assets/active_robot.json')
    if active['host'] != profile['host']:
        raise ValueError('Active robot and server profile disagree')
    return dict(status='numerical_checks_passed', default_server=profile, active_robot=active,
                model_snapshot=snapshot, mapping_sha256={g: digest(ROBOT / f'configs/{g}_mapping.json') for g in mappings},
                calibration_files_sha256=reference_fingerprint(), joints=joints, hands=hands,
                neck_motion_reference_deg=neck['neck'],
                physical_alignment_status='requires_robot_specific_direction_and_zero_evidence',
                live_driver_directions_verified=False, live_startup_targets_verified=False,
                network_access=False, serial_access=False, robot_commands_sent=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=__import__('pathlib').Path)
    args = parser.parse_args()
    result = run()
    if args.out:
        write(args.out, result)
    print(__import__('json').dumps(dict(status=result['status'], server=result['default_server']['host'],
          mapped_joints=len(result['joints']), hand_joints=sum(len(h['joints']) for h in result['hands'].values()),
          physical_alignment_status=result['physical_alignment_status'], report=str(args.out) if args.out else None),
          ensure_ascii=False, indent=2))
