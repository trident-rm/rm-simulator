#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check staged CAD planning, guarded playback, determinism and bounded failure.

Levels 1/2 must complete. Level 3 must either complete or report its bounded
planning failure without moving the live fixture. This never counts a rejected
path as a completed assembly. Requires the generated v5 source-CAD package.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile


def run(binary, package, *args):
    result = subprocess.run([str(binary), '--assembly-assets', str(package), '--headless', *args],
                            text=True, capture_output=True, timeout=300)
    state = json.loads(result.stdout)
    assert result.returncode == (0 if state['cores'][0]['step'] == 'Confirmed' else 1), result.stderr
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--binary', type=Path, default=Path('target/nix/debug/examples/engineer_assembly'))
    parser.add_argument('--out', type=Path, default=Path('local-assets/assembly-cad/planner-scenarios.json'))
    args = parser.parse_args()
    binary, package = args.binary.resolve(), args.package.resolve()
    cases = []
    for difficulty in [1, 2, 3]:
        with tempfile.TemporaryDirectory(prefix='assembly-plan-') as tmp:
            recording = Path(tmp) / 'frames.jsonl'
            state = run(binary, package, '--difficulty', str(difficulty), '--record', str(recording))
            lines = [json.loads(line) for line in recording.read_text().splitlines()]
        planner = state['planner']
        complete = state['cores'][0]['step'] == 'Confirmed'
        assert complete or (difficulty == 3 and planner['status'] == 'failed'), state
        if complete:
            assert planner['status'] == 'ready' and not state['motion_blocked'], state
            assert state['cad']['collision'] is None and lines[-1]['completed']
            assert state['cad']['peak_joint_step_rad'] < 0.1, state
            for frame in lines[1:]:
                cad = frame['cad']
                assert cad['collision'] is None
                assert cad['position_error_m'] <= 1e-5 and cad['angle_error_rad'] <= 1e-4
                if cad['gripped']:
                    w, x, y, z = cad['tool']['rotation_wxyz']
                    axis = [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
                    expected = [p-a*cad['grasp_axial_m'] for p, a in zip(cad['tool']['translation_m'], axis)]
                    assert max(abs(a-b) for a, b in zip(expected, cad['unit_pose']['translation_m'])) < 1e-10
                for pose in frame['frames_world_flu'].values():
                    assert all(math.isfinite(x) for x in pose['translation_m']+pose['rotation_wxyz'])
        else:
            assert state['now_ns'] == 0 and state['cad'] == lines[1]['cad'], state
            assert planner['failure']['reason'] and planner['failure']['statistics']['collision_checks'] <= 8000
        cases.append(dict(difficulty=difficulty, completed=complete, state=state))
    repeated = run(binary, package, '--difficulty', '1')
    assert repeated == cases[0]['state'], 'same planning query must replay identically'
    rejected = run(binary, package, '--difficulty', '1', '--plan-budget', '1')
    assert rejected['planner']['status'] == 'failed' and rejected['now_ns'] == 0
    assert rejected['planner']['failure']['statistics']['collision_checks'] <= 1
    assert not rejected['cad']['gripped']
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(dict(cases=cases, deterministic=True, bounded_failure=True), indent=2)+'\n')
    print(json.dumps(dict(completed=[c['difficulty'] for c in cases if c['completed']],
                          planning_failed=[c['difficulty'] for c in cases if not c['completed']],
                          deterministic=True, bounded_failure=True, report=str(args.out))))


if __name__ == '__main__':
    main()
