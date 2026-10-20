#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check persistent Energy Unit grip, gravity, pickup and collision in the live game.

Start a source PIE Heavy with assembly assets and --console --start-paused.
This test uses the already deployed Unit. It never creates or places an object.
It leaves the settled Unit loose. --recover also tries a bounded guarded approach.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import time
from console import ConsoleClient


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def rotate(q, p):
    return multiply(multiply(q, [0., *p]), [q[0], -q[1], -q[2], -q[3]])[1:]


def compose(a, b):
    offset = rotate(a['rotation_wxyz'], b['translation_m'])
    return {'translation_m': [x+y for x, y in zip(a['translation_m'], offset)],
            'rotation_wxyz': multiply(a['rotation_wxyz'], b['rotation_wxyz'])}


def inverse(p):
    q = [p['rotation_wxyz'][0], *[-v for v in p['rotation_wxyz'][1:]]]
    return {'translation_m': rotate(q, [-v for v in p['translation_m']]), 'rotation_wxyz': q}


def wrist(arm):
    pose = {'translation_m': [0., 0., 0.], 'rotation_wxyz': [1., 0., 0., 0.]}
    for axis, angle in zip(arm['model']['engineer']['axes'], arm['joints_rad']):
        q = [math.cos(angle/2), *[v*math.sin(angle/2) for v in axis['axis']]]
        offset = rotate(q, axis['origin_m'])
        pose = compose(pose, {'translation_m': [a-b for a, b in zip(axis['origin_m'], offset)], 'rotation_wxyz': q})
    return compose(pose, arm['model']['engineer']['tool'])


def approach_resting_unit(client, owner, identity):
    # Observed object feedback only: no fixture placement and no object motion
    # command. The real host still solves IK and checks every requested jog.
    for _ in range(24):
        state = client.call({'type': 'state'})
        arm = next(a for a in state['arms'] if a['owner'] == owner)
        unit = next(u for u in state['energy_units'] if u['physical']['id'] == identity)
        local_unit = compose(inverse(state['chassis']['pose']), unit['physical']['pose'])
        actual = wrist(arm)
        bore = rotate(local_unit['rotation_wxyz'], [0., 0., 1.])
        axis = rotate(actual['rotation_wxyz'], [0., 0., 1.])
        goal = [v+.02*d for v, d in zip(local_unit['translation_m'], bore)]
        error = [a-b for a, b in zip(goal, actual['translation_m'])]
        dot = sum(a*b for a, b in zip(axis, bore))
        if math.sqrt(sum(v*v for v in error)) < .003 and dot > math.cos(math.radians(3)):
            return state
        cross = [axis[1]*bore[2]-axis[2]*bore[1], axis[2]*bore[0]-axis[0]*bore[2], axis[0]*bore[1]-axis[1]*bore[0]]
        delta = [1.+dot, *cross]
        length = math.sqrt(sum(v*v for v in delta))
        if length < 1e-6:
            raise RuntimeError('resting Unit needs a different wrist approach')
        w, x, y, z = [v/length for v in delta]
        # XYZ Euler increments match the host's robot-local jog convention.
        angles = [math.atan2(2*(w*x-y*z), 1-2*(x*x+y*y)),
                  math.asin(max(-1., min(1., 2*(x*z+w*y)))),
                  math.atan2(2*(w*z-x*y), 1-2*(y*y+z*z))]
        client.call({'type': 'world', 'command': {'EnergyArmJog': {'chassis': owner,
            'translation_m': [max(-.015, min(.015, v)) for v in error],
            'rotation_rad': [max(-.08, min(.08, v)) for v in angles]}}})
        for _ in range(16):
            state = client.call({'type': 'step', 'ticks': 16})
            arm = next(a for a in state['arms'] if a['owner'] == owner)
            if max(abs(a-b) for a, b in zip(arm['target_rad'], arm['joints_rad'])) < 1e-5:
                break
    raise RuntimeError('resting Unit remained outside the gripper approach budget')

def run(client, args):
    state = client.call({'type': 'ready'})
    client.call({'type': 'pause', 'paused': True})
    client.call({'type': 'release_inputs'})
    state = client.call({'type': 'step', 'ticks': 256})
    owner = state['chassis_id']
    deadline = time.monotonic() + 30
    while not state.get('energy_units'):
        if time.monotonic() > deadline:
            raise RuntimeError('source Unit did not initialize')
        state = client.call({'type': 'state'})
    # Lift the existing cargo clear of the deck before dropping it. A Unit
    # already resting against the deck need not fall when the jaws open.
    for _ in range(2):
        client.call({'type': 'world', 'command': {'EnergyArmJog': {
            'chassis': owner, 'translation_m': [0., 0., .02], 'rotation_rad': [0., 0., 0.]}}})
        state = client.call({'type': 'step', 'ticks': 64})
    held = next(u for u in state['energy_units'] if u['attachment'] == {'Held': owner})
    identity = held['physical']['id']
    initial_pose = held['physical']['pose']
    assert not state['assemblies'], 'run this test on a fresh pilot'

    def unit(state):
        return next(u for u in state['energy_units'] if u['physical']['id'] == identity)

    def grip():
        return client.call({'type': 'world', 'command': {'EnergyGrip': {'chassis': owner}}})

    def same_pose(a, b):
        position = math.dist(a['translation_m'], b['translation_m'])
        qa, qb = a['rotation_wxyz'], b['rotation_wxyz']
        rotation = min(math.dist(qa, qb), math.dist(qa, [-v for v in qb]))
        return position < 1e-7 and rotation < 1e-6

    if args.screenshots:
        client.call({'type': 'key', 'key': 'j', 'pressed': True})
        client.call({'type': 'key', 'key': 'j', 'pressed': False})
        client.call({'type': 'screenshot', 'path': str(args.out.with_suffix('.held.png').resolve())})
    state = grip()
    assert unit(state)['attachment'] == 'Free'
    assert same_pose(unit(state)['physical']['pose'], initial_pose), (initial_pose, unit(state)['physical']['pose'])
    state = client.call({'type': 'step', 'ticks': 4})
    falling_pose = unit(state)['physical']['pose']
    assert falling_pose['translation_m'][2] < initial_pose['translation_m'][2]
    state = grip()
    assert unit(state)['attachment'] == {'Held': owner}
    assert same_pose(unit(state)['physical']['pose'], falling_pose), 'pickup snapped the Unit'
    picked_grasp = unit(state)['grasp']
    state = client.call({'type': 'step', 'ticks': 32})
    assert unit(state)['attachment'] == {'Held': owner}
    carried_pose = unit(state)['physical']['pose']
    assert unit(state)['grasp'] == picked_grasp, 'carried transform changed'
    state = grip()
    assert unit(state)['attachment'] == 'Free'
    state = client.call({'type': 'step', 'ticks': 256})
    for _ in range(6):
        physical = unit(state)['physical']
        if math.sqrt(sum(v*v for v in physical['velocity_m_s'])) < .05 and math.sqrt(sum(v*v for v in physical['angular_velocity_rad_s'])) < .5:
            break
        state = client.call({'type': 'step', 'ticks': 128})
    resting = unit(state)
    assert resting['attachment'] == 'Free'
    assert resting['physical']['pose']['translation_m'][2] < carried_pose['translation_m'][2] - .01
    assert all(math.isfinite(v) for v in resting['physical']['velocity_m_s'])
    assert math.sqrt(sum(v*v for v in resting['physical']['velocity_m_s'])) < .1, 'Unit did not settle on collision geometry'
    assert len(state['energy_units']) == 1, 'release/pickup created another Unit'
    try:
        client.call({'type': 'world', 'command': {'AutoAssembly': {'chassis': owner, 'difficulty': 1}}})
    except RuntimeError as error:
        assert 'Pick up an Energy Unit' in str(error), str(error)
    else:
        raise AssertionError('assembly created a replacement instead of requiring a held Unit')
    if args.screenshots:
        client.call({'type': 'screenshot', 'path': str(args.out.with_suffix('.dropped.png').resolve())})
    recovered = False if args.recover else None
    recovery_reason = None
    if args.recover:
        try:
            state = approach_resting_unit(client, owner, identity)
            resting_before_pickup = unit(state)['physical']['pose']
            state = grip()
            assert unit(state)['attachment'] == {'Held': owner}
            assert same_pose(unit(state)['physical']['pose'], resting_before_pickup), 'resting pickup snapped the Unit'
            recovered = True
        except RuntimeError as error:
            recovery_reason = str(error)
    report = {'unit_id': identity, 'initial_pose': initial_pose, 'falling_pickup_pose': falling_pose,
              'resting_pose': resting['physical']['pose'], 'resting_velocity_m_s': resting['physical']['velocity_m_s'],
              'collision_max_expansion_m': resting['physical']['shape']['max_expansion_m'],
              'hull_vertices': sum(map(len, resting['physical']['shape']['hulls_m'])),
              'same_identity': True, 'pickup_without_snapping': True, 'resting_pickup': recovered, 'recovery_reason': recovery_reason, 'assembly_without_unit_rejected': True}
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7790)
    parser.add_argument('--screenshots', action='store_true')
    parser.add_argument('--recover', action='store_true', help='Also try a bounded Cartesian approach to the settled object; record reach/path failures.')
    parser.add_argument('--out', type=Path, default=Path('local-assets/energy-unit-game.json'))
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + 30
    while True:
        try:
            connection = socket.create_connection(('127.0.0.1', args.port), 120)
            break
        except ConnectionRefusedError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.1)
    with connection:
        with connection.makefile('rwb') as stream:
            run(ConsoleClient(stream, emit=lambda _: None), args)


if __name__ == '__main__':
    main()
