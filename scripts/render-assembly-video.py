#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Render offline workbench IK poses with unchanged source CAD geometry.

Requires rm-map-tools' Python environment, diagnostic rasterizer, clang++ and
FFmpeg. This renders recorded forward kinematics; planner status comes from the recording.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import hashlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

# Native GLB left/up/forward -> world FLU.
C = np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])


def matrix(pose):
    w, x, y, z = pose['rotation_wxyz']
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    result[:3, 3] = pose['translation_m']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools', type=Path, default=Path('../rm-map-tools'))
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--area', type=Path, help='Verified terrain extraction required by area recordings')
    parser.add_argument('--motion', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--closeup', action='store_true')
    parser.add_argument('--side', action='store_true', help='Orthographic side view for checking the P arc')
    parser.add_argument('--mechanism-only', action='store_true', help='Hide Engineer CAD to expose the Core mechanism')
    args = parser.parse_args()
    sys.path.insert(0, str(args.tools.resolve() / 'python'))
    from gltf_scene import read_glb, mesh_instances, scene_nodes
    records = [json.loads(line) for line in args.motion.read_text().splitlines()]
    header, rows = records[0], records[1:]
    if header['schema'] != 1 or header['tick_ns'] != 7812500 or not rows[-1]['completed']:
        raise ValueError('requires a completed 128 Hz offline IK recording')
    groups = {}
    for key in ['engineer', 'core-base', 'core-arm', 'core-head', 'unit']:
        entry = header['manifest']['files'][key]
        path = args.package / entry['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError(f'recording/source mismatch: {path}')
        document, binary = read_glb(path)
        reference = {document['nodes'][i]['name'][5:]: m
                     for i, m, _ in scene_nodes(document)
                     if document['nodes'][i].get('name', '').startswith('link/')}
        for ni, _, points, triangles, material in mesh_instances(document, binary):
            name = document['nodes'][ni]['extras']['assembly_group']
            frame = reference[name]
            local = (points - frame[:3, 3]) @ frame[:3, :3]
            # Local frame axes are native glTF, just like recorded poses converted below.
            factor = document['materials'][material].get('pbrMetallicRoughness', {}).get('baseColorFactor', [1., 1., 1., 1.])
            linear = np.array(factor[:3])
            rgb = np.where(linear <= .0031308, linear * 12.92, 1.055 * linear ** (1 / 2.4) - .055)
            groups.setdefault(name, []).append((local[triangles] @ C.T, np.tile(rgb, (len(triangles), 1))))
    groups = {name: (np.concatenate([v[0] for v in parts]), np.concatenate([v[1] for v in parts]))
              for name, parts in groups.items()}
    area = header.get('area') or rows[0]['cad'].get('area')
    if area:
        if args.area is None:
            raise ValueError('area recording requires --area; cannot render with missing terrain')
        evidence = json.loads((args.area/'area.json').read_text())
        if evidence != area or hashlib.sha256((args.area/'area.glb').read_bytes()).hexdigest() != area['visual_sha256']:
            raise ValueError('recording/area checksum mismatch')
        doc, buf = read_glb(args.area/'area.glb')
        parts = []
        for _, _, points, triangles, material in mesh_instances(doc, buf):
            factor = doc['materials'][material].get('pbrMetallicRoughness', {}).get('baseColorFactor', [1., 1., 1., 1.])
            linear = np.array(factor[:3])
            rgb = np.where(linear <= .0031308, linear*12.92, 1.055*linear**(1/2.4)-.055)
            parts.append((points[triangles]@C.T,np.tile(rgb,(len(triangles),1))))
        groups['area'] = (np.concatenate([p[0] for p in parts]),np.concatenate([p[1] for p in parts]))
    width, height, fps = 1280, 800, 32
    eye, center, scale = ([2.5, -1.65, 1.65], [.68, .1, .5], 470.)
    if args.closeup:
        eye, center, scale = ([1.4, -.9, 1.1], [.53, .08, .65], 1200.)
    if args.side:
        eye, center = ([.75, -2., .75], [.75, 0., .75])
    forward = np.array(center) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1.]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    camera = np.column_stack([right, up, forward])
    # Fit the complete motion; close-ups fit the unit and gripper, not the whole chassis.
    bounds, group_corners = [], {}
    for name, (tri, _) in groups.items():
        if args.mechanism_only and name.startswith('engineer/'):
            continue
        if name == 'area':
            continue  # Keep the camera fitted to the robot/task, with terrain as context.
        if args.closeup and name not in {'unit/unit', 'core/q', 'engineer/j6', 'engineer/jaw-left', 'engineer/jaw-right'}:
            continue
        low, high = tri.min(axis=(0, 1)), tri.max(axis=(0, 1))
        group_corners[name] = np.array([[x, y, z] for x in [low[0], high[0]]
                                       for y in [low[1], high[1]] for z in [low[2], high[2]]])
    for row in rows:
        for name, corners in group_corners.items():
            frame = matrix(row['frames_world_flu'][name])
            bounds.append((corners @ frame[:3, :3].T + frame[:3, 3]) @ camera)
    bounds = np.concatenate(bounds)
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    center = ((low + high) / 2) @ camera.T
    scale = min((width - 100) / (high[0] - low[0]), (height - 200) / (high[1] - low[1]))
    light = -forward + .8 * up - .3 * right; light /= np.linalg.norm(light)
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 23)
        small = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 18)
    except OSError:
        font, small = ImageFont.load_default(size=23), ImageFont.load_default(size=18)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    # Keep the actual time spacing; repeated terminal samples only add a viewing hold.
    ticks = np.array([row['tick'] for row in rows])
    samples = [rows[min(np.searchsorted(ticks, tick, side='right') - 1, len(rows) - 1)]
               for tick in range(0, int(ticks[-1]) + 4, 4)]
    samples = [rows[0]] * fps + samples + [rows[-1]] * fps
    with tempfile.TemporaryDirectory(prefix='rm-assembly-video-') as temporary:
        folder = Path(temporary); helper = folder / 'raster'
        subprocess.run(['clang++', '-O3', str(args.tools / 'benchmarks/2026-10-17-base-analysis/visibility-raster.cpp'), '-o', str(helper)], check=True)
        encoder = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
                                    '-preset', 'medium', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.out)], stdin=subprocess.PIPE)
        for index, row in enumerate(samples):
            posed, colors = [], []
            for name, (tri, rgb) in groups.items():
                if args.mechanism_only and name.startswith('engineer/'):
                    continue
                frame = np.eye(4) if name == 'area' else matrix(row['frames_world_flu'][name])
                posed.append(tri @ frame[:3, :3].T + frame[:3, 3]); colors.append(rgb)
            geometry, rgb = np.concatenate(posed), np.concatenate(colors)
            normal = np.cross(geometry[:, 1] - geometry[:, 0], geometry[:, 2] - geometry[:, 0])
            normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-20)
            shade = np.clip(rgb * (.55 + .45 * abs(normal @ light))[:, None] * 255, 0, 255).astype('uint8')
            projected = (geometry - center) @ camera
            projected[:, :, 2] *= -1  # diagnostic rasterizer keeps the largest depth
            projected[:, :, 0] = projected[:, :, 0] * scale + width / 2
            projected[:, :, 1] = -projected[:, :, 1] * scale + height / 2 + 25
            data = np.empty(len(geometry), dtype=[('p', '<f4', (3, 3)), ('c', 'u1', (3,))])
            data['p'], data['c'] = projected, shade
            (folder / 'input.bin').write_bytes(struct.pack('<iii', width, height, len(geometry)) + data.tobytes())
            subprocess.run([str(helper), str(folder / 'input.bin'), str(folder / 'frame.ppm'), str(folder / 'owners.bin')], check=True)
            image = Image.open(folder / 'frame.ppm'); draw = ImageDraw.Draw(image)
            if args.side and 'p_origin_world_m' in row['cad']:
                def screen(p):
                    v = (np.array(p) - center) @ camera
                    return (v[0] * scale + width / 2, -v[1] * scale + height / 2 + 25)
                px, py = screen(row['cad']['p_origin_world_m'])
                draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 211, 75))
                draw.text((px + 10, py - 20), 'P (fixed)', font=small, fill=(255, 211, 75))
                trace = [screen(r['cad']['post_tip_world_m']) for r in rows if r['tick'] <= row['tick']]
                if len(trace) > 1:
                    draw.line(trace, fill=(65, 203, 227), width=2)
            draw.rectangle([0, 0, width, 86], fill=(24, 32, 42))
            case = 'fixed case' if header.get('pose_seed') is None else f"seed {header['pose_seed']} / selection {header['pose_index']}"
            draw.text((24, 15), f'PIE Engineer + RMUC 2027 Tech Core | {case} | offline IK', font=font, fill='white')
            p, q, _ = np.degrees(row['angles_rad'])
            grip = 'HELD' if row['cad'].get('gripped') else ('CLOSING' if row['cad'].get('grip_requested') else 'OPEN')
            spin = row['cad'].get('rod_spin_rad')
            twist = '' if spin is None else f' | Twist {math.degrees(spin):.1f} deg'
            draw.text((24, 50), f"{row['tick'] / 128:5.2f} s | {row['step']} | P {p:.1f} deg | Q {q:.1f} deg{twist} | {grip}", font=small, fill=(185, 213, 229))
            draw.rectangle([0, height - 40, width, height], fill=(24, 32, 42))
            note = 'Engineer hidden | ' if args.mechanism_only else ''
            draw.text((24, height - 30), note + ('Source CAD | 128 Hz | collision-checked staged plan | no contact forces' if header.get('planner', {}).get('status') == 'ready' else 'Source CAD | 128 Hz prescribed kinematics | no contact / collision-free planning'), font=small, fill=(205, 215, 225))
            encoder.stdin.write(image.tobytes())
            if index in [fps, len(samples) // 2, len(samples) - 1]:
                image.save(args.out.with_name(args.out.stem + f'-{index:04d}.png'))
            if index % 32 == 0:
                print(f'{index}/{len(samples)} frames', flush=True)
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError('FFmpeg failed')
    print(args.out)


if __name__ == '__main__':
    main()
