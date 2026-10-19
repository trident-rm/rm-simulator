#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise all source-CAD workbench sequences and reject a modified calibration.

Runs the same 128 Hz Cartesian controller and serial IK as the rendered example.
This validates prescribed kinematics, not collision/force or full Heavy gameplay.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package',type=Path,required=True)
    p.add_argument('--binary',type=Path,default=Path('target/nix/debug/examples/engineer_assembly'))
    p.add_argument('--out',type=Path,default=Path('local-assets/assembly-cad/scenarios.json'))
    a=p.parse_args();root=a.package.resolve();binary=a.binary.resolve();results=[]
    calibration=json.loads((root/'assembly.json').read_text())
    w,*v=calibration['core']['tool']['rotation_wxyz']
    def rotate(a):
        dot=sum(x*y for x,y in zip(v,a));vv=sum(x*x for x in v)
        cross=[v[1]*a[2]-v[2]*a[1],v[2]*a[0]-v[0]*a[2],v[0]*a[1]-v[1]*a[0]]
        return [(w*w-vv)*a[i]+2*dot*v[i]+2*w*cross[i] for i in range(3)]
    normal,up=rotate([1,0,0]),rotate([0,0,1])
    e=calibration['core']['tool']['translation_m']
    expected=[e[i]-.154*normal[i]+.208*up[i] for i in range(3)]
    assert max(abs(a-b) for a,b in zip(expected,calibration['core_p_origin_m']))<1e-10
    for difficulty in [1,2,3]:
        with tempfile.TemporaryDirectory(prefix='rm-assembly-recording-') as temporary:
            recording=Path(temporary)/'motion.jsonl'
            r=subprocess.run([str(binary),'--assembly-assets',str(root),'--headless','--difficulty',str(difficulty),
                              '--record',str(recording)],text=True,capture_output=True,check=True)
            records=[json.loads(line) for line in recording.read_text().splitlines()]
        header,frames=records[0],records[1:]
        assert header['tick_ns']==7812500 and header['sample_ticks']==4
        assert frames[0]['tick']==0 and frames[-1]['completed']
        assert not frames[0]['cad']['gripped'] and frames[-1]['cad']['gripped']
        assert frames[-1]['cad']['jaw_angles_rad']==[j['closed_rad'] for j in calibration['engineer_jaws']]
        assert all(0<=b['tick']-a['tick']<=4 for a,b in zip(frames,frames[1:]))
        axes=calibration['engineer']['axes']
        hinge=frames[0]['cad']['p_origin_world_m']
        for frame in frames:
            assert max(abs(a-b) for a,b in zip(hinge,frame['cad']['p_origin_world_m']))<1e-10
            if frame['cad']['gripped']:
                assert frame['cad']['unit_pose']['translation_m']==frame['cad']['tool']['translation_m']
            else:
                assert frame['cad']['unit_pose']==frames[0]['cad']['unit_pose']
            assert frame['cad']['position_error_m']<1e-5
            assert frame['cad']['angle_error_rad']<1e-4
            assert len(frame['frames_world_flu'])==20
            for value,axis in zip(frame['cad']['engineer_q_rad'],axes):
                assert axis['range_rad'][0]<=value<=axis['range_rad'][1]
            for pose in frame['frames_world_flu'].values():
                assert all(math.isfinite(v) for v in pose['translation_m']+pose['rotation_wxyz'])
                assert abs(sum(v*v for v in pose['rotation_wxyz'])-1)<1e-8
        if difficulty==3:
            assert {'Approach','Insert','Lift','Raise','Align','Confirmed'} <= {f['step'] for f in frames}
        state=json.loads(r.stdout)
        assert state['failure'] is None and state['cores'][0]['step']=='Confirmed',state
        assert state['cad']['position_error_m']<1e-5,state
        assert state['cad']['angle_error_rad']<1e-4,state
        assert state['cad']['peak_joint_step_rad']<0.1,state
        assert abs(state['cad']['engineer_q_rad'][3])<=3.141592653589793/2,state
        results.append(dict(difficulty=difficulty,elapsed_s=state['now_ns']/1e9,state=state))
    seeded=[]
    for difficulty,seed in [(2,0),(3,17)]:
        command=[str(binary),'--assembly-assets',str(root),'--headless','--difficulty',str(difficulty),'--pose-seed',str(seed)]
        r=subprocess.run(command,text=True,capture_output=True,check=True)
        state=json.loads(r.stdout)
        repeated=subprocess.run(command,text=True,capture_output=True,check=True)
        assert state==json.loads(repeated.stdout), 'same selection must replay identically'
        generated=json.loads(subprocess.run([str(binary),'--generate-poses',str(seed)],text=True,capture_output=True,check=True).stdout)
        assert state['cad']['requested_pose']==generated[difficulty-1]['cores'][0]
        assert state['target_q_rad'][0]==generated[difficulty-1]['q_rad'][0]
        assert state['cores'][0]['step']=='Confirmed' and not state['ik_unreachable']
        seeded.append(dict(difficulty=difficulty,seed=seed,state=state))
    rejected=subprocess.run([str(binary),'--assembly-assets',str(root),'--headless','--difficulty','3','--pose-seed','1'],text=True,capture_output=True)
    assert rejected.returncode!=0 and 'pedestal-front plane; no resampling performed' in rejected.stderr
    with tempfile.TemporaryDirectory(prefix='rm-assembly-integrity-') as temporary:
        altered=Path(temporary)
        for file in root.iterdir():
            if file.name in {'manifest.json','assembly.json'}:shutil.copyfile(file,altered/file.name)
            elif file.is_file(): (altered/file.name).symlink_to(file)
        config=altered/'assembly.json';config.write_bytes(config.read_bytes()+b'\n')
        r=subprocess.run([str(binary),'--assembly-assets',str(altered),'--headless'],text=True,capture_output=True)
        assert r.returncode!=0 and 'checksum mismatch: assembly.json' in r.stderr,r.stderr
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(dict(cases=results,seeded_cases=seeded,overhang_rejected=True,modified_calibration_rejected=True),indent=2)+'\n')
    print(json.dumps(dict(completed=[r['difficulty'] for r in results],seeded_completed=[(r['difficulty'],r['seed']) for r in seeded],overhang_rejected=True,modified_calibration_rejected=True,report=str(a.out))))


if __name__=='__main__':main()
