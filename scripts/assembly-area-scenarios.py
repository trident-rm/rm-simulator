#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check real-area elevations, stance, source integrity and bounded failure.

Requires the generated rig and terrain packages. A bounded planning rejection
is not counted as assembly completion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def ground(mesh, x, y):
    heights = []
    for triangle in mesh['triangles']:
        a, b, c = [mesh['points_m'][i] for i in triangle]
        den = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den) < 1e-12:
            continue
        u = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den
        v = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den
        z = u*a[2]+v*b[2]+(1-u-v)*c[2]
        if min(u, v, 1-u-v) >= -1e-7 and z <= .6:
            heights.append(z)
    return max(heights)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package', type=Path, required=True)
    p.add_argument('--area', type=Path, required=True)
    p.add_argument('--binary', type=Path, default=Path('target/nix/debug/examples/engineer_assembly'))
    a = p.parse_args()
    evidence = json.loads((a.area/'area.json').read_text())
    for name, key in [('area.glb', 'visual_sha256'), ('terrain.json', 'collision_sha256')]:
        assert hashlib.sha256((a.area/name).read_bytes()).hexdigest() == evidence[key]
    assert evidence['schema'] == 2
    frame = evidence['assembly_frame']['translation_m']
    wall = evidence['assembly_frame_evidence']
    assert abs(wall['measured_span_m']-wall['expected_span_m']) < .01
    assert abs(frame[2]-.25) < 1e-5
    mesh = json.loads((a.area/'terrain.json').read_text())
    bottom, deck = ground(mesh, 0, 0), ground(mesh, 1.4, -.511338)
    assert abs(bottom) < 1e-5 and abs(deck-bottom-.25) < 1e-5, (bottom, deck)
    def run(area, *args):
        return subprocess.run([str(a.binary.resolve()), '--assembly-assets', str(a.package.resolve()),
                               '--assembly-area', str(area.resolve()), *args], text=True, capture_output=True, timeout=60)
    result = run(a.area, '--headless', '--difficulty', '3', '--plan-budget', '1')
    assert result.returncode == 1, result.stderr
    state = json.loads(result.stdout)
    assert state['now_ns'] == 0 and state['planner']['status'] == 'failed'
    assert not state['cad']['gripped'] and state['cad']['area'] == evidence
    assert .383 < state['cad']['engineer_body']['translation_m'][2] < .384
    assert state['cad']['requested_pose']['position_m'] == [-.05, .2, .5]
    with tempfile.TemporaryDirectory(prefix='rm-assembly-area-') as temp:
        corrupt = Path(temp)/'area'
        shutil.copytree(a.area, corrupt)
        with (corrupt/'terrain.json').open('a') as f:
            f.write(' ')
        rejected = run(corrupt, '--collision-audit')
        assert rejected.returncode != 0 and 'checksum mismatch' in rejected.stderr, rejected.stderr
    print(json.dumps(dict(deck_above_plinth_m=deck-bottom, source_checksums=True,
                          supported_stance=True, tampering_rejected=True, bounded_failure=True)))


if __name__ == '__main__':
    main()
