"""Behavioral regressions for phase timing, critique evidence, posture tools and revisions."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,copy,json,shutil,tempfile,zipfile
from pathlib import Path
from unittest.mock import patch
import h5py,numpy as np
from common import SKILL,WORK,read,write,digest,refresh_manifest
import motion,visual_review,catalog,revisions,state_audit
from trajectory import phase_curve,retime_clock
from design_tools import compose
from pose_tools import measure_package,solve


def rejects(fn,kind=ValueError):
    try:fn()
    except kind:return
    raise AssertionError('Expected operation to be rejected')


def run(full=False):
    checks=[]
    curve=phase_curve(np.arange(5.),np.array([[0],[1],[2],[2],[0.]]),[True,False,True,True,True])
    assert abs(curve(1,1)[0])>.1
    for t in [1,2,3]:
        for derivative in [0,1,2]:np.testing.assert_allclose(curve(t-1e-7,derivative),curve(t+1e-7,derivative),atol=3e-5)
    np.testing.assert_allclose(curve(np.linspace(2,3,10)),2,atol=1e-12)
    np.testing.assert_allclose(curve([0,2,3,4],1),0,atol=1e-12)
    checks.append('pass_points_C2_nonzero_velocity_and_explicit_static_stops')
    t=np.linspace(0,6,601);q=np.column_stack([np.where(t<3,(1-np.cos(np.pi*t/3))/2,1),np.zeros(len(t))])
    nt,nq,clock=retime_clock(t,q,2,[[3,6]])
    np.testing.assert_allclose([nt[-1],clock(6)-clock(3)],[4.5,3],atol=1e-12)
    np.testing.assert_array_equal(nq,q)
    rejects(lambda:retime_clock(t,q,2,[[1,3]]))
    rejects(lambda:retime_clock(t,q,float('nan')))
    checks.append('speed_changes_keep_protected_hold_and_reject_moving_hold_ranges')
    with tempfile.TemporaryDirectory(prefix='workflow_test_',dir=WORK) as folder:
        tmp=Path(folder);request=read(SKILL/'assets/design_composition_example.json')
        spec=compose(request);spec['package']='fb_workflow_test_v1'
        write(tmp/'design.json',spec);draft=motion.create(tmp/'design.json',tmp/spec['package'])
        motion.kinematics(draft);refresh_manifest(draft)
        from offline_backend import validate
        validate(draft)
        with h5py.File(draft/'fullbody.hdf5') as f:
            assert not len(f['hand_left/q']) and not len(f['eyou/q'])
            for key in ['arms/cmd_left','arms/cmd_right','neck/q','hand_right/q']:
                values=f[key][:];np.testing.assert_allclose(values[-301:],np.tile(values[-1],(301,1)),atol=1e-10)
        bad=copy.deepcopy(request);bad['components'].append(bad['components'][0])
        rejects(lambda:compose(bad))
        fixed=copy.deepcopy(spec);fixed['package']='fb_fixed_reject_v1';fixed['timing']['fixed_duration']=True;fixed['speed_caps_deg_s']={'arms':.01}
        write(tmp/'fixed.json',fixed);rejects(lambda:motion.create(tmp/'fixed.json',tmp/fixed['package']))
        checks.append('overlapping_arm_hand_head_phrases_backend_units_and_fixed_duration_failure')
        pose=measure_package(draft,0,'right',tmp/'pose.jpg' if full else None)['measurements']
        np.testing.assert_allclose(np.linalg.norm(pose['palm_normal_world']),1)
        target=dict(side='right',palm_position_m=pose['positions_m']['palm'],palm_normal_world=pose['palm_normal_world'],j7_tolerance_deg=.5)
        solutions=solve(target,tmp/'candidates.jpg' if full else None)
        assert len(solutions['candidates'])==3
        assert solutions['candidates'][0]['position_error_m']<.002
        assert all(not c['can_export'] for c in solutions['candidates'])
        checks.append('actual_model_landmarks_and_wrist_bounded_candidate_search_never_certifies_a_path')
        audit=state_audit.snapshot();write(tmp/'snapshot.json',audit)
        assert not state_audit.snapshot(tmp/'snapshot.json')['changed_files']
        altered=copy.deepcopy(audit);altered['files_sha256']['assets/server_profile.json']='changed';write(tmp/'changed.json',altered)
        assert state_audit.snapshot(tmp/'changed.json')['changed_files']==['assets/server_profile.json']
        write(tmp/'wrong_runtime.json',dict(host='wrong',source='fixture',observed_at_utc='fixture'))
        rejects(lambda:state_audit.snapshot(runtime=tmp/'wrong_runtime.json'))
        checks.append('offline_dependency_differences_and_wrong_robot_runtime_evidence_rejection')
        # Same-name replacement preserves real bytes; injected failure restores them.
        def local_fixture(label):
            base=tmp/label;old=base/'packages/fb_same_v1';new=base/'drafts/fb_same_v1';old.mkdir(parents=True);new.mkdir(parents=True)
            (old/'data').write_bytes(b'old');(new/'data').write_bytes(b'new')
            with zipfile.ZipFile(old.with_suffix('.zip'),'w') as z:z.write(old/'data',old.name+'/data')
            return old,new
        old,new=local_fixture('replace_ok')
        with patch('revisions.WORK',tmp):result=revisions.replace_local(new,old)
        assert (old/'data').read_bytes()==b'new' and (Path(result['backup'])/old.name/'data').read_bytes()==b'old'
        old,new=local_fixture('replace_failure');rename=os.rename
        def failing_rename(src,dst):
            if Path(dst)==old.with_suffix('.zip') and '.publish_' in str(src):raise OSError('Injected commit failure')
            return rename(src,dst)
        with patch('revisions.WORK',tmp),patch('revisions.os.rename',side_effect=failing_rename):rejects(lambda:revisions.replace_local(new,old),OSError)
        assert (old/'data').read_bytes()==b'old';revisions.archive_matches(old,old.with_suffix('.zip'))
        checks.append('same_name_local_revision_archives_original_and_rolls_back_injected_failure')
        # Review mechanics use a short synthetic video, explicitly a test fixture.
        # This never counts as an aesthetic review of a robot motion.
        import cv2
        writer=cv2.VideoWriter(str(draft/'preview.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(160,120))
        assert writer.isOpened()
        for i in range(81):writer.write(np.full((120,160,3),i,dtype=np.uint8))
        writer.release();(draft/'keyframes.jpg').write_bytes(b'test fixture, not inspected')
        visual_review.prepare(draft,[0,2,4,8])
        assert motion.review(draft,'','test')['status']=='pending'
        rejects(lambda:motion.review(draft,'fixture','test','passed'))
        assessment=dict(method='Synthetic fixture only, testing evidence mechanics',viewed_files=['review_sheet.jpg'],
            observations=[dict(time_s=2,aspect='fixture',observation='Synthetic test frame only',verdict='needs_changes')])
        assert motion.review(draft,'fixture','test','needs_changes',assessment)['status']=='needs_changes'
        rejects(lambda:motion.review(draft,'fixture','test','passed',assessment))
        assessment['observations'][0]['verdict']='acceptable'
        motion.review(draft,'Fixture review, not robot approval','test','passed',assessment)
        before=(draft/'preview.mp4').read_bytes();(draft/'preview.mp4').write_bytes(before+b'x')
        rejects(lambda:motion.review(draft,'fixture','test','passed',assessment))
        (draft/'preview.mp4').write_bytes(before)
        visual_review.compare(draft,draft,tmp/'comparison.jpg',[0,.5,1])
        db=tmp/'events.json';catalog.record(draft,'user_accepted','Synthetic fixture; not real user approval',db)
        data=catalog.build([draft.parent],database=db,deployments=tmp/'empty')
        row=next(r for r in data['actions'] if r['package']==draft.name)
        assert row['user_accepted'] and not row['physical_tested'] and not row['upload_receipts']
        m=read(draft/'motion.json');m['name']='changed meaning';write(draft/'motion.json',m)
        row=next(r for r in catalog.build([draft.parent],database=db,deployments=tmp/'empty')['actions'] if r['package']==draft.name)
        assert not row['user_accepted'] and row['assistant_review']=='stale_or_missing'
        checks.append('pending_and_negative_reviews_block_pass_stale_media_and_changed_content_invalidate_attestations')
        if full:
            # Real render and full path test, leaving review honestly pending.
            m['name']=spec['name'];write(draft/'motion.json',m);motion.check(draft);motion.render(draft,fps=12)
            visual_review.prepare(draft,[0,1.5,3,5,8])
            motion.review(draft,'Real images prepared; no aesthetic acceptance claimed by this automated test','test','pending')
            rejects(lambda:motion.publish(draft))
            checks.append('real_model_complete_collision_and_video_with_pending_review_export_rejected')
            assessment=dict(method='Synthetic acceptance fixture tests export mechanics only; no actual aesthetic assessment',
                viewed_files=['review_sheet.jpg'],observations=[dict(time_s=1.5,aspect='test fixture',observation='Synthetic report for deleted integration fixture',verdict='acceptable')])
            motion.review(draft,'Synthetic fixture only, not user or robot approval','test','passed',assessment)
            result=motion.publish(draft,tmp/'packages'/draft.name)
            revisions.archive_matches(Path(result['package']),Path(result['archive']))
            assert len(read(draft/'review_history.json'))>=3
            # New evidence is bound to design changes even when q stays equal.
            design=read(draft/'design.json');design['name']='modified design';write(draft/'design.json',design);refresh_manifest(draft)
            rejects(lambda:motion.verify_export(draft))
            checks.append('new_workflow_publish_archive_consistency_and_design_change_invalidation')
    return dict(status='passed',full=full,checks=checks,network_access=False,robot_commands_sent=False,
        synthetic_visual_assessment_only_inside_deleted_test_fixture=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--full',action='store_true');a=p.parse_args();result=run(a.full)
    write(WORK/'verification/workflow_test.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))
