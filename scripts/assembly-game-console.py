#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise automatic L1/L2/L3 through the running app's ordinary host protocol.

Start the source PIE Heavy with assembly assets, --console, --start-paused and
--window-mode headless. Park manually first, or explicitly request the reference
operator fixture with --reference-parking. This script does not alter planner
parking policy. The real game never teleports a Heavy for assembly.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import time
from console import ConsoleClient


def run(client, args):
    state = client.call({'type': 'ready'})
    client.call({'type': 'pause', 'paused': True})
    client.call({'type': 'release_inputs'})
    if args.reference_parking:
        # Explicit operator-only fixture, derived from the approved source case.
        # Production requests always use the player's current chassis placement.
        area = json.loads((args.area / 'area.json').read_text())
        m = area['arena_to_workbench']
        local = [1.278000049957, -.547914948432, .383150523114]
        cad = [sum(m[j][i] * (local[j] - m[j][3]) for j in range(3)) for i in range(3)]
        position = [cad[0], cad[1] - 1.6243436, cad[2] - args.floor_source_z_m]
        yaw = 195. - math.degrees(math.atan2(m[1][0], m[0][0]))
        try:
            client.call({'type': 'spawn', 'position_m': position, 'yaw_deg': yaw})
        except RuntimeError as error:
            # Placement can invalidate an already queued old-life pilot input.
            if 'another life' not in str(error):
                raise
        client.call({'type': 'release_inputs'})
        state = client.call({'type': 'step', 'ticks': 256})
    chassis = state['chassis_id']
    if chassis is None:
        raise RuntimeError('requires an Engineer pilot')
    client.call({'type': 'key', 'key': 'j', 'pressed': True})
    client.call({'type': 'key', 'key': 'j', 'pressed': False})
    args.out.mkdir(parents=True, exist_ok=True)
    results = []
    for level in [1, 2, 3]:
        started = time.monotonic()
        client.call({'type': 'world', 'command': {'AutoAssembly': {'chassis': chassis, 'difficulty': level}}})
        deadline = time.monotonic() + 90
        positioning_ticks=0
        while True:
            state = client.call({'type': 'state'})
            job = next(r for r in state['assemblies'] if r['owner'] == chassis)
            if job['phase'] == 'Positioning':
                assert job['checks'] == 0, 'robot solver ran before Core arrival'
                state=client.call({'type':'step','ticks':16})
                positioning_ticks+=16
                continue
            if job['phase'] != 'Solving':
                break
            if time.monotonic() > deadline:
                raise RuntimeError('planning timed out')
            time.sleep(.1)
        elapsed = time.monotonic() - started
        assert job['phase'] == 'Actuating', job['message']
        if args.screenshots:
            client.call({'type': 'screenshot', 'path': str((args.out / f'l{level}-ready.png').resolve())})
        twist_started=None
        held=None
        while job['phase'] in ['Actuating','AwaitingTarget','SolvingTwist']:
            if job['phase']=='Actuating':
                if held is not None:
                    actual=next(a for a in state['arms'] if a['owner']==chassis)
                    assert actual['joints_rad']==held['joints_rad'],'twist solve moved the arm'
                    held=None
                state=client.call({'type':'step','ticks':min(64,len(job['frames'])-job['cursor'])})
            else:
                assert job['twist_target_rad'] is not None
                assert abs(job['pose']['p_rad']-math.pi/2)<1e-5
                assert job['pose']['gripped']
                if twist_started is None:twist_started=time.monotonic()
                if time.monotonic()-twist_started>90:raise RuntimeError('twist planning timed out')
                actual=next(a for a in state['arms'] if a['owner']==chassis)
                if held is None:held=actual
                assert actual['joints_rad']==held['joints_rad']
                time.sleep(.05)
                state=client.call({'type':'state'})
            job=next(r for r in state['assemblies'] if r['owner']==chassis)
        assert job['phase'] == 'Completed', job['message']
        if args.screenshots:
            client.call({'type': 'screenshot', 'path': str((args.out / f'l{level}-complete.png').resolve())})
        result = {'level': level, 'positioning_ticks': positioning_ticks, 'solving_s': elapsed, 'frames': len(job['frames']), 'checks': job['checks'], 'phase': job['phase'], 'revealed_target_rad':job['twist_target_rad']}
        results.append(result)
        print(json.dumps(result), flush=True)
    client.call({'type': 'world', 'command': {'AutoAssembly': {'chassis': chassis, 'difficulty': 3}}})
    client.call({'type': 'world', 'command': {'CancelAutoAssembly': {'chassis': chassis}}})
    state = client.call({'type': 'state'})
    assert next(r for r in state['assemblies'] if r['owner'] == chassis)['phase'] == 'Cancelled'
    (args.out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7790)
    parser.add_argument('--reference-parking', action='store_true')
    parser.add_argument('--screenshots', action='store_true')
    parser.add_argument('--area', type=Path, default=Path('local-assets/assembly-cad/area'))
    parser.add_argument('--floor-source-z-m', type=float, default=-1.640343675041)
    parser.add_argument('--out', type=Path, default=Path('local-assets/assembly-cad/in-game'))
    args = parser.parse_args()
    with socket.create_connection(('127.0.0.1', args.port), 120) as connection:
        with connection.makefile('rwb') as stream:
            run(ConsoleClient(stream, emit=lambda _: None), args)


if __name__ == '__main__':
    main()
