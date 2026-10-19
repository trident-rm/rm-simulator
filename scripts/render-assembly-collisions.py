#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Render original collision triangles and named source leaves from a pinned audit.

Run the ignored audit_chassis_shoulder_collision_leaves workbench test first.
Requires rm-map-tools' CAD Python environment and diagnostic CPU rasterizer.
This is an inspection render, never a completed assembly recording.
"""
import argparse
import hashlib
import importlib.util
import json
import re
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

C = np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])
PLOT = np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def world(points, pose):
    w, x, y, z = pose['rotation_wxyz']
    return points @ Rotation.from_quat([x, y, z, w]).as_matrix().T + pose['translation_m']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tools', type=Path, default=Path('../rm-map-tools'))
    p.add_argument('--package', type=Path, required=True)
    p.add_argument('--audit', type=Path, required=True)
    p.add_argument('--hierarchy', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--first-part', type=int, help='Explicit first-frame leaf for inspecting a cleared historical pose')
    p.add_argument('--compare-deployed', action='store_true', help='Compare a rejected pose with its audited source-camera posture')
    a = p.parse_args()
    audit = json.loads(a.audit.read_text())
    if audit['schema'] != 1 or audit['manifest_sha256'] != digest(a.package/'manifest.json') or audit['collision_sha256'] != digest(a.package/'collision.json'):
        raise ValueError('audit/package source mismatch')
    asset = json.loads((a.package/'collision.json').read_text())
    if digest(a.package/'engineer.glb') != asset['source_glbs']['engineer.glb']:
        raise ValueError('collision/visual source mismatch')
    sys.path.insert(0, str(a.tools.resolve()/'python'))
    from gltf_scene import read_glb, mesh_instances, scene_nodes
    spec = importlib.util.spec_from_file_location('raster', a.tools/'benchmarks/2026-10-17-base-passes/validate_render.py')
    raster = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(raster)
    doc, buf = read_glb(a.package/'engineer.glb')
    frames = {doc['nodes'][i]['name'][5:]: m for i, m, _ in scene_nodes(doc)
              if doc['nodes'][i].get('name', '').startswith('link/')}
    bodies = {b['frame']: b['parts'] for b in asset['bodies'] if b['frame'].startswith('engineer/')}
    hierarchy = {r['path']: r['name'] for r in json.loads(a.hierarchy.read_text())}
    metadata, geometry, counts = {}, {}, {}
    for ni, _, points, triangles, _ in mesh_instances(doc, buf):
        e = doc['nodes'][ni]['extras']
        group = e['assembly_group']
        index = counts.get(group, 0)
        counts[group] = index + 1
        part = bodies[group][index]
        frame = frames[group]
        local = ((points-frame[:3, 3]) @ frame[:3, :3]) @ C.T
        exact = np.asarray(part['points_m'])[part['triangles']]
        if not np.allclose(exact, local[triangles], atol=1e-12, rtol=0):
            raise ValueError(f'collision/source triangles differ: {group} leaf {index}')
        key = (group, index)
        geometry[key] = exact
        paths = e.get('source_occurrence_paths', [])
        metadata[key] = dict(frame=group, part=index, source_solid=e.get('native_solid_index'),
                             paths=paths, names=[hierarchy.get(v, '?') for v in paths])
    if any(counts[k] != len(v) for k, v in bodies.items()):
        raise ValueError('source-leaf accounting mismatch')
    # Core leaf ordering is arm convex leaves followed by exact head meshes,
    # matching the exporter. Draw the actual collision hull where one is used.
    all_bodies = {b['frame']: b['parts'] for b in asset['bodies']}
    core_groups = {audit.get('first_frame', ''), audit.get('second_frame', '')}
    core_groups = {g for g in core_groups if g.startswith('core/')}
    for file in ['core-base.glb', 'core-arm.glb', 'core-head.glb'] if core_groups else []:
        if digest(a.package/file) != asset['source_glbs'][file]:
            raise ValueError('Core collision/visual source mismatch')
        document, binary = read_glb(a.package/file)
        reference = {document['nodes'][i]['name'][5:]: m for i,m,_ in scene_nodes(document)
                     if document['nodes'][i].get('name', '').startswith('link/')}
        for ni, _, points, triangles, _ in mesh_instances(document, binary):
            e = document['nodes'][ni]['extras']
            group = e['assembly_group']
            if group not in core_groups:
                continue
            index = counts.get(group, 0)
            counts[group] = index+1
            part = all_bodies[group][index]
            frame = reference[group]
            local = ((points-frame[:3,3])@frame[:3,:3])@C.T
            cloud = np.asarray(part['points_m'])
            if part['kind'] == 'mesh':
                tri = cloud[part['triangles']]
                if not np.allclose(tri, local[triangles], atol=1e-12, rtol=0):
                    raise ValueError('Core mesh/source mismatch')
            else:
                from scipy.spatial import ConvexHull, cKDTree
                if np.max(cKDTree(local).query(cloud)[0]) > 1e-12:
                    raise ValueError('Core hull/source vertex mismatch')
                tri = cloud[ConvexHull(cloud).simplices]
            key = (group,index)
            geometry[key] = tri
            shell = e.get('native_shell_index')
            metadata[key] = dict(frame=group,part=index,kind=part['kind'],source_shell=shell,
                                 paths=[f'{file}:shell-{shell}'],names=[f'Core source shell {shell}'])
    if any(counts[g] != len(all_bodies[g]) for g in core_groups):
        raise ValueError('Core source-leaf accounting mismatch')
    first = audit.get('first_frame', 'engineer/body')
    second = audit.get('second_frame', 'engineer/j2')
    source = next(c for c in audit['cases'] if c['label'] == 'source-zero')
    rejected = next(c for c in sorted(audit['cases'], key=lambda c: not c['label'].startswith('planner-')) if (c['label'].startswith('planner-') or c['label'].startswith('initial-branch')) and (c['intersecting_leaves'] or a.first_part is not None))
    collision_case = rejected
    if a.compare_deployed:
        source = rejected
        rejected = next(c for c in audit['cases'] if c['label'] == source['label']+'-aux-deployed')
    culprit = a.first_part if a.first_part is not None else collision_case['intersecting_leaves'][0]['body_part']
    red = (first, culprit)
    pair_metadata = [metadata[red]] + [metadata[(second, i)] for i in sorted({h['j2_part'] for h in collision_case['intersecting_leaves']})]
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 24)
        small = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 19)
    except OSError:
        font, small = ImageFont.load_default(size=24), ImageFont.load_default(size=19)
    a.out.mkdir(parents=True, exist_ok=True)
    panels = []
    views = [(.8, .5, 1.1), (-.9, .35, .8), (1., .15, -.1)]
    with tempfile.TemporaryDirectory(prefix='assembly-collision-') as folder:
        tmp = Path(folder)
        helper = tmp/'raster'
        subprocess.run(['clang++', '-O3', str(a.tools/'benchmarks/2026-10-17-base-analysis/visibility-raster.cpp'), '-o', str(helper)], check=True)
        for case in [source, rejected]:
            posed = {k: world(v, case['frames'][k[0]]) @ PLOT.T for k, v in geometry.items()}
            for mode in ['context', 'isolated', 'joint']:
                selected = [k for k in geometry if mode == 'context' or k == red or k[0] == second]
                tri = np.concatenate([posed[k] for k in selected])
                colors = np.concatenate([np.tile([1., .23, .08] if k == red else [.12, .77, .94] if k[0] == second else [.50, .54, .59], (len(posed[k]), 1)) for k in selected])
                # Isolate the colliding surfaces rather than fitting unrelated Core geometry.
                reference = tri
                if mode == 'joint':
                    # Deliberate viewport crop centred on the suspect part.
                    # The original triangles remain intact, including offscreen ones.
                    center = (posed[red].min((0, 1))+posed[red].max((0, 1)))/2
                    if (case is rejected or a.compare_deployed) and collision_case['intersecting_leaves']:
                        overlap = np.asarray(collision_case['intersecting_leaves'][0]['overlap_bounds_m'])
                        center = overlap.mean(axis=0)@PLOT.T
                    corners = np.array([[x,y,z] for x in [-.065,.065] for y in [-.065,.065] for z in [-.065,.065]])+center
                    reference = corners.reshape(-1,1,3)
                for view_index, view in enumerate(views):
                    rendered = Image.fromarray(raster.raster(tri, colors, reference, view, helper, tmp))
                    image = Image.new('RGB', (1200, 1200), (192, 208, 224))
                    image.paste(rendered.resize((1020, 1020), Image.Resampling.LANCZOS), (90, 115))
                    draw = ImageDraw.Draw(image)
                    draw.rectangle((0, 0, 1200, 95), fill=(24, 32, 42))
                    label = ('Stowed camera' if case is source else 'Source CAD camera posture') if a.compare_deployed else ('Zero-joint reference' if case is source else f'Audited IK pose: {len(case["intersecting_leaves"])} intersecting leaf pairs')
                    draw.text((24, 15), label[:65] + {'isolated': ' | isolated collision parts', 'joint': ' | joint close-up (cropped)', 'context': ' | robot context'}[mode], font=font, fill='white')
                    draw.text((24, 53), f'Orange: {",".join(metadata[red]["paths"])} / {first}   Cyan: {second}', font=small, fill=(210, 223, 234))
                    draw.rectangle((0, 1152, 1200, 1200), fill=(24, 32, 42))
                    draw.text((24, 1166), 'Source collision meshes (Core convex leaves where declared) | PIE / H3 + DJI CAD', font=small, fill=(210, 223, 234))
                    name = re.sub(r'[^a-zA-Z0-9_.-]+', '-', f'{case["label"]}-{mode}-{view_index}.png')
                    image.save(a.out/name)
                    panels.append(dict(case=case['label'], mode=mode, view=view_index, file=name))
    result = dict(schema=1, audit_sha256=digest(a.audit), suspect=metadata[red], colliding_sources=pair_metadata,
                  source_triangle_agreement_m=1e-12, rejected_joints_rad=rejected['q_rad'], selected_case_intersections=len(rejected['intersecting_leaves']), panels=panels,
                  policy='Exact source collision geometry at audited frames. Zero-joint reference retains stowed auxiliary mechanisms. Orange marks the selected source leaf; cyan marks the second link. No renderer-side geometry changes.')
    (a.out/'review.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
