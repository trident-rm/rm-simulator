#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Record a scripted source-station retrieval through ordinary live arm commands.

Requires an embedded PIE Heavy on console 7790 and a connected free-camera
spectator on console 7791. Operator parking happens before recording. The runner never assigns Unit
poses, attachments or force parameters; ordinary commands and physics own them.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from console import ConsoleClient

_spec = importlib.util.spec_from_file_location('unit_helpers', Path(__file__).with_name('energy-unit-game-console.py'))
unit_math = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(unit_math)


def rotation_delta(target, actual):
    q = unit_math.multiply(target, [actual[0], *[-v for v in actual[1:]]])
    if q[0] < 0:
        q = [-v for v in q]
    w, x, y, z = q
    return [math.atan2(2*(w*x-y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(x*z+w*y)))),
            math.atan2(2*(w*z-x*y), 1-2*(y*y+z*z))]


def run(owner, camera, args, temporary):
    state = owner.call({'type': 'ready'})
    owner.call({'type': 'pause', 'paused': True})
    owner.call({'type': 'release_inputs'})
    if args.park:
        try:
            owner.call({'type': 'spawn', 'position_m': [13.11, 6.45, 1.0], 'yaw_deg': 0.})
        except RuntimeError as error:
            if 'another life' not in str(error):
                raise
        state = owner.call({'type': 'step', 'ticks': 256})
    else:
        state = owner.call({'type': 'state'})
    chassis = state['chassis_id']
    if chassis is None or not any(a['owner'] == chassis for a in state['arms']):
        raise RuntimeError('requires a source PIE Heavy pilot')
    stock = [u for u in state['energy_units'] if u['attachment'] == 'Mounted' and u['physical']['pose']['translation_m'][0] > 0]
    if len(stock) != 6:
        raise RuntimeError('requires a fresh red station with all six Units mounted')
    selected = max(stock, key=lambda u: u['physical']['pose']['translation_m'][2])
    identity = selected['physical']['id']
    mounted = selected['physical']['pose']
    point = mounted['translation_m']
    axis = unit_math.rotate(mounted['rotation_wxyz'], [0., 0., 1.])
    audit = {'source': 'rm-simulator GPU screenshots; ordinary EnergyArmJog/EnergyGrip commands',
             'operator_parking': args.park, 'unit_id': identity, 'mount': selected['supply'],
             'parking_pose': state['chassis']['pose'], 'trace': []}
    frames = []
    previous_tick = state['tick']
    phase = 'Approach'
    if not args.dry_run:
        camera.call({'type': 'ready'})
        camera.call({'type': 'resize', 'width': 1920, 'height': 1080})
        camera.call({'type': 'capture', 'captured': True})
        camera.call({'type': 'camera', 'position_m': [point[0]-1.3, point[1]-1., point[2]+.65],
                     'yaw_deg': 33., 'pitch_deg': -22.})

    def current_unit():
        return next(u for u in state['energy_units'] if u['physical']['id'] == identity)

    def capture(duration):
        if args.dry_run:
            return
        deadline = time.monotonic()+10
        while camera.call({'type': 'state'})['tick'] < state['tick']:
            if time.monotonic() > deadline:
                raise RuntimeError('spectator did not receive the stepped checkpoint')
        path = temporary / f'{len(frames):05}.png'
        camera.call({'type': 'screenshot', 'path': str(path)})
        label = phase
        if phase == 'Force-limited withdrawal' and current_unit()['attachment'] == {'Held': chassis}:
            label = 'Extracted: prescribed carry'
        frames.append((path, duration, label, current_unit()['attachment'], current_unit()['supply']['pull']['travel_m']))

    def step(ticks):
        nonlocal state, previous_tick
        remaining = ticks
        while remaining:
            count = remaining if args.dry_run else min(remaining, max(1, round(128/args.fps)))
            state = owner.call({'type': 'step', 'ticks': count})
            unit = current_unit()
            audit['trace'].append({'phase': phase, 'tick': state['tick'], 'attachment': unit['attachment'],
                                   'pose': unit['physical']['pose'], 'pull': unit['supply']['pull'],
                                   'joints_rad': arm()['joints_rad'], 'jaws_rad': arm()['jaws_rad'],
                                   'chassis_pose': state['chassis']['pose'], 'wrist_pose': world_wrist(),
                                   'grasp': unit['grasp']})
            capture((state['tick']-previous_tick)/128)
            previous_tick = state['tick']
            remaining -= count

    def arm():
        return next(a for a in state['arms'] if a['owner'] == chassis)

    def world_wrist():
        return unit_math.compose(state['chassis']['pose'], unit_math.wrist(arm()))

    def move_to(position, rotation=None):
        nonlocal state
        for iteration in range(160):
            actual = unit_math.wrist(arm())
            target = unit_math.compose(unit_math.inverse(state['chassis']['pose']),
                                       {'translation_m': position, 'rotation_wxyz': rotation or world_wrist()['rotation_wxyz']})
            error = [a-b for a, b in zip(target['translation_m'], actual['translation_m'])]
            angular = rotation_delta(target['rotation_wxyz'], actual['rotation_wxyz'])
            if math.sqrt(sum(v*v for v in error)) < .002 and max(abs(v) for v in angular) < .002:
                return
            distance_step = .0015 if phase == 'Force-limited withdrawal' else .012
            scale = min(1., distance_step/max(1e-12, math.sqrt(sum(v*v for v in error))))
            command = {'EnergyArmJog': {'chassis': chassis, 'translation_m': [v*scale for v in error],
                                       'rotation_rad': [max(-.06, min(.06, v)) for v in angular]}}
            print(phase, iteration, [round(v, 4) for v in world_wrist()['translation_m']], flush=True)
            state = owner.call({'type': 'world', 'command': command})
            for _ in range(32):
                step(4)
                if max(abs(a-b) for a, b in zip(arm()['target_rad'], arm()['joints_rad'])) < 1e-5:
                    break
            else:
                raise RuntimeError('arm did not reach its bounded joint target')
        raise RuntimeError('Cartesian waypoint exceeded the jog budget')

    capture(.75)
    wrist = world_wrist()
    # Move clear of the other five Units before raising the open gripper.
    move_to([point[0]-.16, wrist['translation_m'][1], wrist['translation_m'][2]])
    aligned = [1.+axis[2], -axis[1], axis[0], 0.]
    length = math.sqrt(sum(v*v for v in aligned))
    aligned = [v/length for v in aligned]
    # The pads fit the 64.6 mm mid-body, not the 95 mm caps: enter from
    # the side along the open fork, rather than threading a cap through it.
    grip_point = [v+.02*d for v, d in zip(point, axis)]
    move_to([point[0]-.16, grip_point[1], grip_point[2]], aligned)
    move_to(grip_point, aligned)
    phase = 'Grip'
    state = owner.call({'type': 'world', 'command': {'EnergyGrip': {'chassis': chassis}}})
    if current_unit()['attachment'] != {'Extracting': chassis}:
        raise RuntimeError('grip did not capture the selected mounted Unit')
    step(96)
    phase = 'Force-limited withdrawal'
    start = world_wrist()
    move_to([v+.085*d for v, d in zip(start['translation_m'], axis)], start['rotation_wxyz'])
    step(64)
    if current_unit()['attachment'] != {'Held': chassis}:
        raise RuntimeError('finite pull did not extract the selected Unit')
    phase = 'Carried'
    wrist = world_wrist()
    move_to([wrist['translation_m'][0]-.10, wrist['translation_m'][1], wrist['translation_m'][2]], wrist['rotation_wxyz'])
    step(64)
    audit['final_attachment'] = current_unit()['attachment']
    audit['stock_count'] = len(state['energy_units'])
    trace = audit['trace']
    for key, name in [('joints_rad', 'max_joint_rad_s'), ('jaws_rad', 'max_jaw_rad_s')]:
        peak = max(max(abs(a-b) for a, b in zip(after[key], before[key])) /
                   ((after['tick']-before['tick'])/128) for before, after in zip(trace, trace[1:]))
        if peak > 1.+1e-8:
            raise RuntimeError(f'{name} exceeded the ordinary actuator limit: {peak}')
        audit[name] = peak
    audit['captured_frames'] = len(frames)
    audit['duration_s'] = sum(frame[1] for frame in frames)
    args.out.with_suffix('.json').write_text(json.dumps(audit, indent=2)+'\n')
    if args.dry_run:
        print('retrieval passed', identity, flush=True)
        return
    concat = temporary/'frames.txt'
    concat.write_text(''.join(f"file '{p}'\nduration {duration:.9f}\n" for p, duration, *_ in frames)+f"file '{frames[-1][0]}'\n")
    subtitles = temporary/'status.srt'
    def stamp(seconds):
        ms = round(seconds*1000)
        return f'{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}'
    clock = 0.
    entries = []
    for i, (_, duration, label, attachment, travel) in enumerate(frames):
        text = f'{label} | magnet: 10 N + gravity | arm cap: 20 N | grip cap: 25 N'
        if label == 'Force-limited withdrawal':
            text += f' | travel: {travel*1000:.1f} mm'
        entries.append(f'{i+1}\n{stamp(clock)} --> {stamp(clock+duration)}\n{text}\n\n')
        clock += duration
    subtitles.write_text(''.join(entries))
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(concat),
                    '-vf',f"crop=1920:800:0:150,scale=1280:-2,pad=1280:720:0:80:black,drawtext=fontfile=/System/Library/Fonts/Supplemental/Arial.ttf:text='RMUC supply retrieval - scripted live capture':fontcolor=white:fontsize=24:x=28:y=28,subtitles={subtitles}:fontsdir=/System/Library/Fonts/Supplemental:force_style='Fontname=Arial,Fontsize=10,Outline=1'",
                    '-r',str(args.fps),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)],check=True)
    print(args.out,flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=7790)
    parser.add_argument('--camera-port',type=int,default=7791)
    parser.add_argument('--fps',type=int,choices=[24,30,60],default=30)
    parser.add_argument('--park',action='store_true',help='Use an operator-only reference parking before capture.')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rm-supply-video-') as temporary:
        with socket.create_connection(('127.0.0.1',args.port),120) as connection:
            with connection.makefile('rwb') as stream:
                owner = ConsoleClient(stream,emit=lambda _:None)
                if args.dry_run:
                    run(owner,None,args,Path(temporary))
                else:
                    with socket.create_connection(('127.0.0.1',args.camera_port),120) as observer:
                        with observer.makefile('rwb') as camera_stream:
                            run(owner,ConsoleClient(camera_stream,emit=lambda _:None),args,Path(temporary))


if __name__ == '__main__':
    main()
