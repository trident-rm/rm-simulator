#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Capture live in-game automatic assembly through the app console, then encode MP4.

Uses the normal GPU-rendered app and host-owned planner, never the offline
workbench rasterizer. Actuation is frame-stepped at 128 Hz and played at normal
game speed. Start a paused source PIE Heavy with assembly assets and --console.
Park manually first, or explicitly use the operator-only --reference-parking
fixture. Intermediate PNGs are removed after encoding; an audit JSON is retained.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from console import ConsoleClient
import importlib.util
_grip_spec=importlib.util.spec_from_file_location('energy_unit_helpers',Path(__file__).with_name('energy-unit-game-console.py'))
_grip_helpers=importlib.util.module_from_spec(_grip_spec)
_grip_spec.loader.exec_module(_grip_helpers)


def capture(client, args, temporary):
    state = client.call({'type': 'ready'})
    client.call({'type': 'pause', 'paused': True})
    client.call({'type': 'release_inputs'})
    if args.reference_parking:
        area = json.loads((args.area / 'area.json').read_text())
        matrix = area['arena_to_workbench']
        body = [1.278000049957, -.547914948432, .383150523114]
        cad = [sum(matrix[j][i] * (body[j] - matrix[j][3]) for j in range(3)) for i in range(3)]
        position = [cad[0], cad[1] - 1.6243436, cad[2] - args.floor_source_z_m]
        yaw = 195. - math.degrees(math.atan2(matrix[1][0], matrix[0][0]))
        try:
            client.call({'type': 'spawn', 'position_m': position, 'yaw_deg': yaw})
        except RuntimeError as error:
            if 'another life' not in str(error):
                raise
        client.call({'type': 'release_inputs'})
        state = client.call({'type': 'step', 'ticks': 256})
    state = client.call({'type': 'capture', 'captured': True})
    if state['ui']['blocks_input']:
        client.call({'type': 'key', 'key': 'escape', 'pressed': True})
        state = client.call({'type': 'key', 'key': 'escape', 'pressed': False})
    owner = state['chassis_id']
    if owner is None:
        raise RuntimeError('requires a source PIE Heavy pilot')
    initial_unit=next((u for u in state.get('energy_units',[]) if u['attachment']=={'Held':owner}),None)
    if initial_unit is None:
        raise RuntimeError('recording requires the existing held Energy Unit')
    unit_id=initial_unit['physical']['id']
    segments = []
    audit = []

    def screenshot(duration):
        path = temporary / f'{len(segments):05}.png'
        client.call({'type': 'screenshot', 'path': str(path)})
        segments.append([path, duration])
        return len(segments) - 1

    if args.show_postures:
        for folded in [True, False]:
            client.call({'type': 'key', 'key': 't', 'pressed': True})
            client.call({'type': 'key', 'key': 't', 'pressed': False})
            for frame in range(args.fps):
                target = round((frame + 1) * 128 / args.fps)
                previous = round(frame * 128 / args.fps)
                state = client.call({'type': 'step', 'ticks': target - previous})
                screenshot(1 / args.fps)
            assert state['arms'][0]['folded'] == folded
    initial_arm = next(a for a in state['arms'] if a['owner'] == owner)
    client.call({'type': 'key', 'key': 'j', 'pressed': True})
    client.call({'type': 'key', 'key': 'j', 'pressed': False})
    screenshot(.75)
    start = time.monotonic()
    state = client.call({'type': 'world', 'command': {'AutoAssembly': {'chassis': owner, 'difficulty': args.level}}})
    positioning_ticks=0
    while True:
        job=next(r for r in state['assemblies'] if r['owner']==owner)
        if job['phase']!='Positioning':break
        assert job['checks']==0,'robot solver ran before Core arrival'
        assert next(a for a in state['arms'] if a['owner']==owner)['joints_rad']==initial_arm['joints_rad']
        screenshot(1/args.fps)
        next_tick=round((round(positioning_ticks*args.fps/128)+1)*128/args.fps)
        count=max(1,next_tick-positioning_ticks)
        state=client.call({'type':'step','ticks':count})
        positioning_ticks+=count
        audit.append({'phase':'Positioning','checks':0,'ticks':positioning_ticks})
    start=time.monotonic()
    previous = None
    while True:
        job = next(r for r in state['assemblies'] if r['owner'] == owner)
        if job['phase'] != 'Solving':
            break
        if time.monotonic() - start > 90:
            raise RuntimeError('planning did not finish within 90 seconds')
        now = time.monotonic()
        if previous is not None:
            index, stamp = previous
            segments[index][1] = max(1 / args.fps, now - stamp)
        index = screenshot(1 / args.fps)
        previous = index, now
        audit.append({'phase': 'Solving', 'checks': job['checks'], 'elapsed_s': now - start})
        state = client.call({'type': 'state'})
    solve_s = time.monotonic() - start
    if previous is not None:
        segments[previous[0]][1] = max(1 / args.fps, time.monotonic() - previous[1])
    if job['phase'] != 'Actuating':
        raise RuntimeError(job['message'])
    actual = next(a for a in state['arms'] if a['owner'] == owner)
    assert actual['joints_rad'] == initial_arm['joints_rad'], 'solve changed actual joints'
    assert actual['jaws_rad'] == initial_arm['jaws_rad'], 'solve changed actual jaws'
    max_joint_rad_s=max_jaw_rad_s=0.
    segment_ticks=[]
    twist_solving_s=0.
    while True:
        actual=next(a for a in state['arms'] if a['owner']==owner)
        previous_q,previous_jaws=actual['joints_rad'],actual['jaws_rad']
        for sample in job['frames']:
            max_joint_rad_s=max(max_joint_rad_s,*(abs(a-b)*128 for a,b in zip(sample['joints_rad'],previous_q)))
            max_jaw_rad_s=max(max_jaw_rad_s,*(abs(a-b)*128 for a,b in zip(sample['jaws_rad'],previous_jaws)))
            previous_q,previous_jaws=sample['joints_rad'],sample['jaws_rad']
        assert max_joint_rad_s<=1.0003 and max_jaw_rad_s<=1.0003
        total=len(job['frames']);segment_ticks.append(total)
        print(f'Segment {len(segment_ticks)}: capturing {total} ticks',flush=True)
        for frame in range(math.ceil(total*args.fps/128)):
            target=min(total,round(frame*128/args.fps))
            if target>job['cursor']:
                state=client.call({'type':'step','ticks':target-job['cursor']})
                job=next(r for r in state['assemblies'] if r['owner']==owner)
            if job['phase'] not in ['Actuating','BreakingAway']:raise RuntimeError(job['message'])
            screenshot(1/args.fps)
            audit.append({'phase':job['phase'],'cursor':job['cursor'],'stage':job['message'],'target_q_rad':job['twist_target_rad']})
        state=client.call({'type':'step','ticks':total-job['cursor']})
        job=next(r for r in state['assemblies'] if r['owner']==owner)
        if job['phase'] in ['Completed','Recovering']:break
        if job['phase'] not in ['AwaitingTarget','SolvingTwist']:raise RuntimeError(job['message'])
        assert job['twist_target_rad'] is not None
        assert abs(job['pose']['p_rad']-math.pi/2)<1e-5 and job['pose']['gripped']
        held=next(a for a in state['arms'] if a['owner']==owner)
        started=time.monotonic();previous=None
        while job['phase'] in ['AwaitingTarget','SolvingTwist']:
            actual=next(a for a in state['arms'] if a['owner']==owner)
            assert actual['joints_rad']==held['joints_rad'] and actual['jaws_rad']==held['jaws_rad']
            now=time.monotonic()
            if now-started>90:raise RuntimeError('twist planning timed out')
            if previous is not None:segments[previous[0]][1]=max(1/args.fps,now-previous[1])
            index=screenshot(1/args.fps);previous=index,now
            audit.append({'phase':job['phase'],'checks':job['checks'],'target_q_rad':job['twist_target_rad']})
            state=client.call({'type':'state'});job=next(r for r in state['assemblies'] if r['owner']==owner)
        twist_solving_s+=time.monotonic()-started
        if previous is not None:segments[previous[0]][1]=max(1/args.fps,time.monotonic()-previous[1])
        if job['phase'] not in ['Actuating','BreakingAway']:raise RuntimeError(job['message'])
        actual=next(a for a in state['arms'] if a['owner']==owner)
        assert actual['joints_rad']==held['joints_rad']
    recovery_ticks=0
    recovery_trace=[]
    neutral=next(a for a in state['arms'] if a['owner']==owner)['joints_rad']
    while job['phase']=='Recovering':
        ticks=max(1,round(128/args.fps))
        state=client.call({'type':'step','ticks':ticks})
        recovery_ticks+=ticks
        if recovery_ticks>8192:raise RuntimeError('Core recovery timed out')
        job=next(r for r in state['assemblies'] if r['owner']==owner)
        unit=next(u for u in state['energy_units'] if u['physical']['id']==unit_id)
        local=_grip_helpers.compose(_grip_helpers.inverse(job['world_from_rig']),unit['physical']['pose'])
        recovery_trace.append({'tick':recovery_ticks,'attachment':unit['attachment'],'position_rig_m':local['translation_m']})
        assert next(a for a in state['arms'] if a['owner']==owner)['joints_rad']==neutral
        screenshot(ticks/128)
    assert job['phase']=='Completed',job['message']
    total=sum(segment_ticks)
    end_joints = next(a for a in state['arms'] if a['owner'] == owner)['joints_rad']
    state = client.call({'type': 'step', 'ticks': 128})
    assert next(a for a in state['arms'] if a['owner'] == owner)['joints_rad'] == end_joints
    final_unit=next(u for u in state['energy_units'] if u['physical']['id']==unit_id)
    assert final_unit['attachment']=='Free' and final_unit['spent'], 'Core did not drop the same spent Unit'
    local=_grip_helpers.compose(_grip_helpers.inverse(job['world_from_rig']),final_unit['physical']['pose'])['translation_m']
    rim=job['model']['assembly_frame']['translation_m']
    assert local[2]<rim[2], ('Unit did not fall below the collection opening',local,rim)
    assert abs(local[0]-(rim[0]-.45))<.2 and abs(local[1]-rim[1])<.25, ('Unit missed collection opening',local,rim)
    screenshot(1.5)
    listing = temporary / 'frames.txt'
    # TemporaryDirectory gives plain generated basenames, without shell quoting.
    listing.write_text(''.join(f"file '{path.name}'\noption framerate {args.fps}\nduration {duration:.9f}\n" for path, duration in segments)
                       + f"file '{segments[-1][0].name}'\noption framerate {args.fps}\n")
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'concat', '-safe', '0',
                    '-i', str(listing), '-vf', f'fps={args.fps},format=yuv420p', '-c:v', 'libx264',
                    '-preset', 'medium', '-crf', '18', '-t', str(sum(duration for _, duration in segments)),
                    '-movflags', '+faststart', str(args.out)], check=True)
    args.out.with_suffix('.json').write_text(json.dumps({
        'source': 'rm-simulator live app console/GPU screenshots', 'level': args.level,
        'operator_reference_parking': args.reference_parking,
        'energy_unit_id':unit_id,'initial_unit_attachment':initial_unit['attachment'],'final_unit_attachment':final_unit['attachment'],
        'positioning_ticks':positioning_ticks, 'show_postures': args.show_postures,
        'max_joint_rad_s': max_joint_rad_s, 'max_jaw_rad_s': max_jaw_rad_s,
        'initial_joints_rad': initial_arm['joints_rad'], 'held_final_joints_rad': end_joints,
        'solving_elapsed_s': solve_s,
        'actuation_ticks': total, 'segment_ticks':segment_ticks, 'twist_solving_s':twist_solving_s, 'revealed_target_rad':job['twist_target_rad'], 'tick_hz': 128, 'fps': args.fps, 'completed': True,
        'core_recovery_ticks':recovery_ticks,'recovery_trace':recovery_trace,'final_unit_spent':final_unit['spent'],'unit_below_collection_rim':True,
        'duration_s': sum(duration for _, duration in segments), 'samples': audit,
    }, indent=2) + '\n')
    print(args.out, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7790)
    parser.add_argument('--level', type=int, choices=[1, 2, 3], default=3)
    parser.add_argument('--fps', type=int, choices=[12, 24, 30, 60], default=30)
    parser.add_argument('--reference-parking', action='store_true')
    parser.add_argument('--show-postures', action='store_true', help='Record T folding and return to neutral first.')
    parser.add_argument('--area', type=Path, default=Path('local-assets/assembly-cad/area'))
    parser.add_argument('--floor-source-z-m', type=float, default=-1.640343675041)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rm-assembly-game-') as directory:
        with socket.create_connection(('127.0.0.1', args.port), 120) as connection:
            with connection.makefile('rwb') as stream:
                capture(ConsoleClient(stream, emit=lambda _: None), args, Path(directory))


if __name__ == '__main__':
    main()
