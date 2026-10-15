#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Add exact source-body armor/joint partitions and fitted detectors to a 2027 package.

Run with rm-map-tools/ocpenv/bin/python. The input stays unchanged; output is a
new local candidate. Base shield travel is an illustrative 0.25 m translation,
not a rulebook or CAD limit. Source module bodies are never reconstructed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys

SOURCE_SHA = 'a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0'
# Exact source-body identities in RMUC2027 V1.0.0. Shared tower frame stays fixed.
OUTPOST_MODULES = [2104330, 2107796, 2200540, 2203298, 2203490,
                   2206254, 2209720, 2302471, 2305243, 2305435,
                   2308214, 2311680, 2404428, 2407197, 2407389]
OUTPOST_SCREENS = [2200540, 2302471, 2404428]
BASE_SCREENS = [4099408, 4121375, 4143351]
BASE_SHIELDS = [4314593, 4353403, 4392221]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def write_glb(path, doc, binary):
    js = json.dumps(doc, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    binary += b'\0' * (-len(binary) % 4)
    path.write_bytes(struct.pack('<III', 0x46546c67, 2, 28 + len(js) + len(binary))
                     + struct.pack('<II', len(js), 0x4e4f534a) + js
                     + struct.pack('<II', len(binary), 0x004e4942) + binary)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--package', type=Path, required=True)
    ap.add_argument('--split', type=Path, required=True)
    ap.add_argument('--map-tools', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    if args.out.exists():
        ap.error('output already exists')
    import numpy as np
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    from validate_parts import read_part
    from mesh_parts import tessellate
    from export_field_package import GlbWriter
    from export_elements import quaternion_xyzw
    from gltf_scene import read_glb
    import provenance
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder
    from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as TopTools_IndexedMapOfShape
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_SHELL
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    manifest = json.loads((args.package / 'manifest.json').read_text())
    if manifest['source_sha256'] != SOURCE_SHA or manifest['season'] != '2027':
        raise ValueError('source identity mismatch')
    parts = json.loads((args.split / 'parts.json').read_text())['parts']
    result = {}
    report = {}
    # Stage only after all source imports and detector fits succeed.
    args.out.parent.mkdir(parents=True, exist_ok=True)
    import tempfile
    with tempfile.TemporaryDirectory(dir=args.out.parent) as directory:
        stage = Path(directory) / 'field'
        shutil.copytree(args.package, stage)
        for name, product, screens, moving in [('outpost', 2603236, OUTPOST_SCREENS, OUTPOST_MODULES),
                                               ('base', 4774244, BASE_SCREENS, BASE_SHIELDS)]:
            part = next(p for p in parts if p['product_id'] == product)
            path = args.split / part['file']
            doc, _, error = read_part(str(path))
            if error:
                raise ValueError(error)
            tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
            roots = TDF_LabelSequence()
            tool.GetFreeShapes(roots)
            compound = TopoDS_Compound()
            builder = BRep_Builder()
            builder.MakeCompound(compound)
            for i in range(1, roots.Length() + 1):
                builder.Add(compound, XCAFDoc_ShapeTool.GetShape_s(roots.Value(i)))
            solids = TopTools_IndexedMapOfShape()
            faces = TopTools_IndexedMapOfShape()
            TopExp.MapShapes_s(compound, TopAbs_SOLID, solids)
            TopExp.MapShapes_s(compound, TopAbs_FACE, faces)
            owners = {}
            ids = set()
            for i in range(1, solids.Extent() + 1):
                solid = solids.FindKey(i)
                box = Bnd_Box()
                BRepBndLib.AddOptimal_s(solid, box, False, False)
                lo, hi = box.CornerMin(), box.CornerMax()
                bounds = np.array([lo.X(),lo.Y(),lo.Z(),hi.X(),hi.Y(),hi.Z()])
                body_faces = TopTools_IndexedMapOfShape()
                TopExp.MapShapes_s(solid, TopAbs_FACE, body_faces)
                candidates = sorted([(float(np.max(np.abs(bounds - np.array(b['bbox_min'] + b['bbox_max'])))), b)
                                     for b in part['bodies'] if b['faces'] == body_faces.Extent()], key=lambda item:item[0])
                # STEP vertex boxes omit extrema inside analytic curved faces;
                # use face count plus a unique nearest box for body identity.
                if not candidates or candidates[0][0] > 10.0 or (len(candidates)>1 and candidates[1][0]-candidates[0][0]<0.05):
                    raise ValueError(f'ambiguous source body identity: {bounds}, {[(d,b["id"]) for d,b in candidates[:2]]}')
                matches = [candidates[0][1]]
                source_id = matches[0]['id']
                ids.add(source_id)
                ex = TopExp_Explorer(solid, TopAbs_FACE)
                while ex.More():
                    fi = faces.FindIndex(ex.Current())
                    if fi in owners and owners[fi] != source_id:
                        raise ValueError('source face belongs to two solids')
                    owners[fi] = source_id
                    ex.Next()
            # One SHELL_BASED_SURFACE_MODEL per source product contains many
            # disconnected shells. Assign all remaining faces together, checking
            # its exact source face count rather than treating shells as solids.
            missing = [b for b in part['bodies'] if b['id'] not in ids]
            remaining = [fi for fi in range(1,faces.Extent()+1) if fi not in owners]
            if missing:
                if len(missing)!=1 or len(remaining)!=missing[0]['faces']:
                    raise ValueError(f'unidentified source surface model: {len(remaining)} faces, {[(b["id"],b["faces"]) for b in missing]}')
                for fi in remaining:
                    owners[fi]=missing[0]['id']
                ids.add(missing[0]['id'])
            if ids != {b['id'] for b in part['bodies']}:
                raise ValueError('incomplete source body coverage')
            triangle_owners = []
            def observed(_, face, count):
                triangle_owners.extend([owners[faces.FindIndex(face)]] * count)
            stats = {}
            vertices, triangles, colors, palette = tessellate(doc, 1.0, 0.35, stats=stats, face_observer=observed)
            if stats.get('unmeshed_faces') or len(triangle_owners) != len(triangles):
                raise ValueError('incomplete source tessellation')
            vertices = vertices / 1000.
            triangle_owners = np.array(triangle_owners)
            color_keys = np.array([palette[c] for c in colors], dtype=object)
            fitted = []
            for source_id in screens:
                # Source armor optics: use the broad outward planar optical face.
                selected = triangles[(triangle_owners == source_id) & np.array([c is not None and (c.startswith('1.0000,0.') or c.startswith('0.0000,1.0000,1.')) for c in color_keys])]
                p = vertices[selected]
                cross = np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0])
                weights = np.linalg.norm(cross, axis=1)
                normal = cross / np.maximum(weights[:, None], 1e-20)
                centers = p.mean(axis=1)
                radial = centers.copy()
                radial[:, 1] = 0.
                outward = (normal * radial).sum(axis=1)
                normal[outward < 0] *= -1
                groups = {}
                for j in range(len(selected)):
                    if weights[j] < 1e-8:
                        continue
                    # Direction bins merge the tessellated, planar optical sheet.
                    key = tuple(np.round(normal[j], 2))
                    groups.setdefault(key, []).append(j)
                key, js = max(groups.items(), key=lambda item: weights[item[1]].sum())
                n = np.average(normal[js], axis=0, weights=weights[js])
                n /= np.linalg.norm(n)
                sheet = p[js].reshape(-1, 3)
                width = np.cross([0., 1., 0.], n)
                width /= np.linalg.norm(width)
                height = np.cross(n, width)
                basis = np.stack([n, width, height], axis=1)
                projected = sheet @ basis
                lo, hi = projected.min(0), projected.max(0)
                center = basis @ ((lo+hi)/2)
                # Include paired coplanar bars, not only the largest single bar.
                all_sheet = vertices[triangles[triangle_owners == source_id]].reshape(-1, 3)
                # The manual does not publish the new dot-matrix active mask.
                # Use the source module's projected envelope as an explicit
                # detector approximation, with its front plane outside the skin.
                bounds2 = all_sheet @ basis
                low, high = bounds2.min(0), bounds2.max(0)
                middle = (low+high)/2
                middle[0] = high[0]+0.0015
                center = basis @ middle
                half = (high-low)[1:]/2
                if np.any(half < .01) or np.any(half > .071):
                    raise ValueError(f'implausible detector {name}/{source_id}: {half}, normal {n}')
                fitted.append({'pose': (center, basis), 'half_m': half.tolist(), 'body': source_id})
            if name == 'outpost':
                # Intersect the projected outward normal lines for the rotor axis.
                rows, rhs = [], []
                for f in fitted:
                    center, basis = f['pose']
                    n = basis[[0, 2], 0]
                    tangent = np.array([-n[1], n[0]])
                    rows.append(tangent)
                    rhs.append(tangent @ center[[0, 2]])
                xz = np.linalg.lstsq(rows, rhs, rcond=None)[0]
                pivot = np.array([xz[0], 1.14, xz[1]])
                groups = [(moving, pivot, np.array([0., 1., 0.]), None, 'outpost.spin')]
            else:
                pivot = None
                groups = []
                for i, body in enumerate(moving):
                    b = next(b for b in part['bodies'] if b['id'] == body)
                    axis = (np.array(b['bbox_min'])+b['bbox_max'])/2
                    axis[1] = 0.
                    axis /= np.linalg.norm(axis)
                    groups.append(([body], np.zeros(3), axis, [0., .25], f'base.armor.{i}'))
            writer = GlbWriter(name, provenance.glb_asset('bind-rmuc2027-structures.py', manifest['source_file'], SOURCE_SHA, frame='source asset axes, metres'))
            node_ids = {}
            for body in sorted(ids):
                select = triangle_owners == body
                origin = next((g[1] for g in groups if body in g[0]), np.zeros(3))
                node_ids[body] = len(writer.nodes)+1
                writer.add_node(f'source_body_{body}', vertices-origin, triangles[select], list(color_keys[select]))
            file = Path('outpost.glb' if name == 'outpost' else 'equipment/base.glb')
            writer.write(stage/file)
            glb, binary = read_glb(stage/file)
            motion = []
            for bodies, origin, axis, limits, identity in groups:
                children = [node_ids[b] for b in bodies]
                glb['nodes'][0]['children'] = [n for n in glb['nodes'][0]['children'] if n not in children]
                origin_node = len(glb['nodes'])
                motion_node = origin_node+1
                glb['nodes'].append({'name':identity+'.origin','translation':origin.tolist(),'children':[motion_node]})
                glb['nodes'].append({'name':identity,'extras':{'rm':{'id':identity}},'children':children})
                glb['nodes'][0]['children'].append(origin_node)
                motion.append({'id':identity,'bodies':[f'source_body_{b}' for b in bodies],
                               'axis':axis.tolist(),'pivot_m':origin.tolist(),'limits':limits})
            write_glb(stage/file, glb, binary)
            collision = file.with_name(file.stem+'-collision.glb')
            shutil.copyfile(stage/file, stage/collision)
            mfile = stage/('manifest.json' if name=='outpost' else 'equipment/manifest.json')
            m = json.loads(mfile.read_text())
            entry = m['assets'][name]
            entry.update(visual_sha256=sha(stage/file), collision_sha256=sha(stage/collision),
                         triangles=len(triangles), collision_triangles=len(triangles), collision_method='source-tessellation-v1')
            entry.pop('mesh_simplification', None)
            write_json(mfile, m)
            result[name] = {'fitted':fitted,'pivot':pivot,'motion':motion,'placements':entry['placements_in_source_arena_frame']}
            report[name] = {'product':product,'source_part_sha256':sha(path),'bodies':sorted(ids),
                            'moving_bodies':moving,'detector_bodies':screens,'triangles':len(triangles),'stats':stats}

        def root(placement):
            x,y,z,w = placement['rotation_xyzw']
            r = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                          [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                          [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            t = np.array(placement['translation_m'])-[0,1.6243436,manifest['floor_top_source_z_m']]
            return r,t
        def detector(f, r, t):
            center, basis = f['pose']
            x,y,z,w = quaternion_xyzw(r@basis)
            return {'pose':{'translation_m':(r@center+t).tolist(),'rotation_wxyz':[w,x,y,z]},'half_m':f['half_m']}
        world = {'bases':[None,None],'outposts':[None,None],'zones':[], 'rotor_direction':1}
        for name, data in result.items():
            for placement in data['placements']:
                r,t=root(placement)
                side = 0 if t[0]>=0 else 1
                ds=[detector(f,r,t) for f in data['fitted']]
                if name=='base':world['bases'][side]=ds
                else:
                    axis=r@np.array([0.,1.,0.])
                    if axis[2]<0.99:
                        raise ValueError('outpost source axis is not approximately up')
                    world['outposts'][side]={'pivot_m':(r@data['pivot']+t).tolist(),'axis':axis.tolist(),'detectors':ds}
        # RFID is populated by the host from the reviewed zone catalog, rather
        # than duplicating those drawings in this exporter.
        bindings={'schema_version':1,'ruleset':'rmuc2027-v1.0.0','source_sha256':SOURCE_SHA,
                  'world':world,'outpost_motion':result['outpost']['motion'],
                  'base_motion':result['base']['motion'],'report':report,
                  'limitations':['Shared tower frame remains fixed; armor-module assemblies rotate.',
                                 'Base shield travel is illustrative 0.25 m, not a measured end stop.',
                                 'Only the three armor modules present in each source base are bound.',
                                 'Detector rectangles use projected source-module envelopes; final dot-matrix masks are unavailable.']}
        write_json(stage/'rmuc2027-bindings.json',bindings)
        for file in ['manifest.json','equipment/manifest.json']:
            m=json.loads((stage/file).read_text())
            if file=='manifest.json':
                m['rmuc2027_bindings']={'file':'rmuc2027-bindings.json','sha256':sha(stage/'rmuc2027-bindings.json')}
            write_json(stage/file,m)
        (stage/'structure-binding-build.json').write_text(json.dumps({'script_sha256':sha(Path(__file__)), 'report':report},indent=2)+'\n')
        shutil.move(str(stage),args.out)
        print(json.dumps({'out':str(args.out),'structures':{k:{'detectors':len(v['fitted']),'triangles':report[k]['triangles']} for k,v in result.items()}},indent=2))


if __name__ == '__main__':
    main()
