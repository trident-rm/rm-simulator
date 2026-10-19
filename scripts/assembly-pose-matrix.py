#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run frozen Core/parking cases with full source-CAD planning and playback.

Failures and source-pose rejections are results, never resampled successes.
Requires the generated assembly package and real Assembly Area extraction.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time


def cases():
    baseline = [-.05, .2, .5, 0., 90., 0.]
    parking = [-.1, -.15, 15.]
    yield dict(name='reference', pose=baseline, parking=parking, q=30.)
    for name, offset in [('farther-50mm', [-.05, -.15, 15.]),
                         ('lateral-50mm', [-.1, -.1, 15.]),
                         ('yaw-minus-15deg', [-.1, -.15, 0.])]:
        yield dict(name=name, pose=baseline, parking=offset, q=30.)
    for name, index, value in [('core-higher-25mm', 2, .525),
                               ('core-lateral-25mm', 1, .225),
                               ('core-azimuth-10deg', 3, 10.),
                               ('core-inclination-80deg', 4, 80.),
                               ('core-roll-20deg', 5, 20.)]:
        pose = baseline.copy()
        pose[index] = value
        yield dict(name=name, pose=pose, parking=parking, q=30.)
    yield dict(name='negative-q', pose=baseline, parking=parking, q=-30.)
    for name, spin in [('locked-clock', [0., 0., 0., 0., 0.]),
                       ('twist-reference', [0., 0., 0., 90., 0.]),
                       ('twist-approach', [22.5, 0., 0., 67.5, 0.]),
                       ('twist-insertion', [0., 22.5, 0., 67.5, 0.]),
                       ('twist-lift', [0., 0., 22.5, 67.5, 0.]),
                       ('twist-alignment', [0., 0., 0., 90., 22.5])]:
        yield dict(name=name, pose=baseline, parking=parking, q=30., spin=spin)
    for seed in [17, 29]:
        yield dict(name=f'seed-{seed}', seed=seed, selection=0, parking=parking)


def validate_recording(path):
    header, *rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert header['planner']['status'] == 'ready' and rows[-1]['completed']
    body = rows[0]['cad']['engineer_body']
    for row in rows:
        cad = row['cad']
        assert cad['fixed_chassis'] and cad['engineer_body'] == body
        assert cad['collision'] is None
        assert cad['position_error_m'] <= 1e-5 and cad['angle_error_rad'] <= 1e-4
        assert cad['peak_joint_step_rad'] < .1
        assert all(math.isfinite(v) for p in row['frames_world_flu'].values()
                   for v in p['translation_m'] + p['rotation_wxyz'])
        if cad['gripped']:
            w, x, y, z = cad['tool']['rotation_wxyz']
            axis = [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
            expected = [p-a*cad['grasp_axial_m'] for p, a in zip(cad['tool']['translation_m'], axis)]
            assert max(abs(a-b) for a, b in zip(expected, cad['unit_pose']['translation_m'])) < 1e-9
            cosine = math.cos(cad['grasp_roll_rad']/2)
            sine = math.sin(cad['grasp_roll_rad']/2)
            expected_q = [w*cosine+z*sine, x*cosine-y*sine,
                          y*cosine+x*sine, z*cosine-w*sine]
            assert abs(sum(a*b for a, b in zip(expected_q, cad['unit_pose']['rotation_wxyz']))) > 1-1e-9
    for a, b in zip(rows, rows[1:]):
        dt = (b['tick']-a['tick']) / 128
        assert abs(b['cad']['rod_spin_rad']-a['cad']['rod_spin_rad']) <= .6*dt+1e-8
    return dict(ticks=rows[-1]['tick'], fixed_body=body,
                cached_guard_replays=rows[-1]['cad'].get('cached_guard_replays', 0),
                spin_range_rad=[min(r['cad']['rod_spin_rad'] for r in rows),
                                max(r['cad']['rod_spin_rad'] for r in rows)],
                route=header['planner']['statistics']['selected_route'])


def run_case(case, args):
    stem = args.out / case['name']
    command = [str(args.binary), '--assembly-assets', str(args.package),
               '--assembly-area', str(args.area), '--difficulty', '3', '--headless',
               '--fixed-chassis', '--chassis-offset', *map(str, case['parking']),
               '--plan-budget', str(args.budget), '--record', str(stem.with_suffix('.jsonl'))]
    if 'pose' in case:
        command += ['--core-pose', *map(str, case['pose']), '--q-deg', str(case['q'])]
    else:
        command += ['--pose-seed', str(case['seed']), '--pose-index', str(case['selection'])]
    if 'spin' in case:
        command += ['--spin-deltas', *map(str, case['spin'])]
    if args.hints:
        command += ['--ik-hints', str(args.hints)]
    started = time.monotonic()
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=args.timeout)
    except subprocess.TimeoutExpired as error:
        stem.with_suffix('.log').write_text(str(error.stderr or ''))
        return dict(case=case, outcome='timeout', elapsed_s=time.monotonic()-started)
    stem.with_suffix('.log').write_text(result.stderr)
    stem.with_suffix('.json').write_text(result.stdout)
    row = dict(case=case, elapsed_s=time.monotonic()-started, exit_code=result.returncode)
    if not result.stdout.strip():
        row.update(outcome='source-or-placement-rejected', reason=result.stderr.strip())
        return row
    state = json.loads(result.stdout)
    row['requested_pose'] = state['cad']['requested_pose']
    row['q_rad'] = state['target_q_rad'][0]
    planner = state['planner']
    if state['cores'][0]['step'] == 'Confirmed':
        assert result.returncode == 0 and not state['motion_blocked']
        row.update(outcome='confirmed', validation=validate_recording(stem.with_suffix('.jsonl')))
    elif planner['status'] == 'failed':
        assert result.returncode == 1 and state['now_ns'] == 0
        row.update(outcome='bounded-planning-failure', failure=planner['failure']['reason'])
    else:
        row.update(outcome='playback-failure', reason=state['diagnostic'])
    row['statistics'] = planner.get('statistics', planner.get('failure', {}).get('statistics'))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('target/nix/debug/examples/engineer_assembly'))
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--area', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=Path('local-assets/assembly-cad/pose-matrix'))
    parser.add_argument('--budget', type=int, default=2500)
    parser.add_argument('--jobs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=900.)
    parser.add_argument('--hints', type=Path, help='Completed source recording used only for IK warm starts.')
    parser.add_argument('--case', action='append', help='Run only these named cases (repeatable).')
    args = parser.parse_args()
    args.binary, args.package, args.area = (p.resolve() for p in (args.binary, args.package, args.area))
    if args.hints:
        args.hints = args.hints.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    selected = [case for case in cases() if not args.case or case['name'] in args.case]
    if not selected or args.jobs < 1 or args.budget < 1 or args.timeout <= 0:
        parser.error('select at least one case and positive execution limits')
    # Keep one executable inode for the whole sweep, even if a developer rebuilds.
    with tempfile.TemporaryDirectory(prefix='assembly-matrix-', dir=args.out) as temporary:
        binary = Path(temporary)/'engineer_assembly'
        try:
            os.link(args.binary, binary)
        except OSError:
            shutil.copy2(args.binary, binary)
        args.binary = binary.resolve()
        hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in
                  [('binary', args.binary), ('package_manifest', args.package/'manifest.json'),
                   ('area_evidence', args.area/'area.json')]}
        report = dict(schema=1, hashes=hashes, budget=args.budget, cases=[])
        if args.hints:
            hashes['ik_hints'] = hashlib.sha256(args.hints.read_bytes()).hexdigest()
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(run_case, case, args) for case in selected]
            for future in as_completed(futures):
                row = future.result()
                report['cases'].append(row)
                report['cases'].sort(key=lambda r: r['case']['name'])
                (args.out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
                print(json.dumps(dict(name=row['case']['name'], outcome=row['outcome'],
                                      elapsed_s=round(row['elapsed_s'], 2))), flush=True)


if __name__ == '__main__':
    main()
