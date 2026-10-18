#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise prescribed robot mechanisms against real 128 Hz physics contacts.

Build first: cargo build -p rm-simulator-physics --example robot_mechanisms.
The stair scenario crosses two 200 mm steps. Its negative control uses the
same drivetrain without deploying the linkage. No poses or velocities are injected.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import tempfile


def run(binary, robot, terrain, commands):
    with tempfile.TemporaryDirectory(prefix='rm-mechanism-') as directory:
        path = Path(directory) / 'commands.csv'
        path.write_text('\n'.join(','.join(map(str, row)) for row in commands)+'\n')
        output = subprocess.check_output([str(binary.resolve()), robot, terrain, str(path)], text=True)
    rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(io.StringIO(output))]
    assert rows and all(math.isfinite(v) for row in rows for v in row.values()), 'nonfinite physics state'
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('target/nix/debug/examples/robot_mechanisms'))
    args = parser.parse_args()
    commands = [(0,0,0),(128,.3,0),(210,1,1),(650,.7,0),(950,1,1),(1400,.7,0),(1580,0,0),(1800,0,0)]
    stairs = run(args.binary,'stairs','steps',commands)
    control = run(args.binary,'stairs','steps',[(t,s,0) for t,s,_ in commands])
    hero = run(args.binary,'hero','flat',[(0,0,0),(128,0,1),(384,0,0),(640,0,0)])
    climb = stairs[-1]; blocked = control[-1]
    assert climb['x_m'] > 2.4 and .66 < climb['z_m'] < .70, climb
    assert climb['contacts'] == 4 and climb['input_rad'] == 0, climb
    assert blocked['x_m'] < .25 and blocked['z_m'] < .4, blocked
    assert max(row['input_rad'] for row in hero) > 1.5 and hero[-1]['input_rad'] == 0, hero[-1]
    print(json.dumps({'two_200mm_steps':climb, 'without_deployment':blocked,
                      'hero_max_angle_rad':max(row['input_rad'] for row in hero),
                      'hero_final_angle_rad':hero[-1]['input_rad']}, indent=2))


if __name__ == '__main__':
    main()
