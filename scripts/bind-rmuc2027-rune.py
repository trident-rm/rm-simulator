#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Fit source Rune detector apertures and bind exact arm subtrees in a new package.

Requires rm-map-tools' CAD Python environment (numpy/scipy). Mesh buffers,
materials and rest geometry remain exact. The scoring disk is the rulebook's
150 mm radius, fitted to each source aperture with 4 mm illustrative exposure.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import struct
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):
    data=path.read_bytes(); size=struct.unpack_from('<I',data,12)[0]
    return json.loads(data[20:20+size]),data[28+size:]
def write(path,doc,binary):
    data=json.dumps(doc,separators=(',',':'),allow_nan=False).encode(); data+=b' '*(-len(data)%4)
    path.write_bytes(struct.pack('<III',0x46546c67,2,28+len(data)+len(binary))+struct.pack('<II',len(data),0x4e4f534a)+data+struct.pack('<II',len(binary),0x004e4942)+binary)
def points(doc,binary,primitive):
    a=doc['accessors'][primitive['attributes']['POSITION']];v=doc['bufferViews'][a['bufferView']]
    if a['componentType']!=5126 or v.get('byteStride',12)!=12:raise ValueError('unexpected source vertex layout')
    return np.frombuffer(binary,dtype='<f4',count=3*a['count'],offset=v.get('byteOffset',0)+a.get('byteOffset',0)).reshape(-1,3).astype(float)
def descendants(doc,index):
    yield index
    for child in doc['nodes'][index].get('children',[]): yield from descendants(doc,child)

def node_matrix(node):
    """Resolve glTF's column-major matrix or T*R*S, including nonuniform scale."""
    if 'matrix' in node:
        if any(k in node for k in ('translation', 'rotation', 'scale')):
            raise ValueError('mixed glTF matrix and TRS')
        return np.array(node['matrix']).reshape(4,4).T
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix()@np.diag(node.get('scale',[1,1,1]))
    matrix[:3,3]=node.get('translation',[0,0,0])
    return matrix

def aperture(doc,binary,arm):
    candidates=[];rng=np.random.default_rng(2027)
    for index in descendants(doc,arm):
        node=doc['nodes'][index]
        for primitive in doc['meshes'][node['mesh']]['primitives'] if 'mesh' in node else []:
            p=np.unique(points(doc,binary,primitive),axis=0)
            if len(p)<24 or len(p)>2000:continue
            origin=p.mean(0);_,_,axes=np.linalg.svd(p-origin,full_matrices=False)
            if np.max(np.abs((p-origin)@axes[2]))>0.00001:continue
            xy=(p-origin)@axes[:2].T
            for _ in range(500):
                q=xy[rng.choice(len(xy),3,replace=False)]; a=2*(q[1:]-q[0])
                if abs(np.linalg.det(a))<1e-10:continue
                c=np.linalg.solve(a,(q[1:]**2).sum(1)-(q[0]**2).sum());r=np.linalg.norm(q[0]-c)
                # Source aperture is 308 mm; the manual's active disk is 300 mm.
                if abs(r-.154)>.0003:continue
                selected=np.abs(np.linalg.norm(xy-c,axis=1)-r)<.0001
                if selected.sum()<12:continue
                ang=np.sort(np.arctan2(*(xy[selected]-c).T[::-1]));gap=np.max(np.diff(np.r_[ang,ang[0]+2*np.pi]))
                if gap>2.1:continue
                v=xy[selected]; fitted=np.linalg.lstsq(np.column_stack([2*v,np.ones(len(v))]),(v*v).sum(1),rcond=None)[0]
                c=fitted[:2]; radius=np.sqrt(fitted[2]+c@c); error=np.max(np.abs(np.linalg.norm(v-c,axis=1)-radius))
                candidates.append((int(selected.sum()),origin+c@axes[:2],axes[2],radius,error,node.get('name'),float(gap)))
    if not candidates:raise ValueError(f'no unique circular source aperture: {doc["nodes"][arm]["name"]}')
    best=max(candidates,key=lambda x:x[0]); centers=np.array([c[1] for c in candidates if c[0]>=best[0]])
    if np.max(np.linalg.norm(centers-best[1],axis=1))>.001:raise ValueError('ambiguous source aperture centers')
    return best

def bind(doc,banks):
    before=copy.deepcopy(doc); selected=[]
    for bank,pivot,axis,team,arms in banks:
        parent=next(i for i,n in enumerate(doc['nodes']) if n.get('name')==bank)
        children=[]; names=[]
        for arm_name in arms:
            arm=next(i for i,n in enumerate(doc['nodes']) if n.get('name')==arm_name)
            if arm not in doc['nodes'][parent]['children']:raise ValueError('unexpected source hierarchy')
            for index in descendants(doc,arm):
                node=doc['nodes'][index]
                if 'mesh' in node:
                    node['name']=f'{arm_name}/mesh-{index}'
                    names.append(node['name'])
            child=doc['nodes'][arm]
            matrix=node_matrix(child)
            matrix[:3,3]-=pivot
            for key in ('translation','rotation','scale'):child.pop(key,None)
            child['matrix']=matrix.T.flatten().tolist();children.append(arm)
        doc['nodes'][parent]['children']=[i for i in doc['nodes'][parent]['children'] if i not in children]
        doc['nodes'][parent]['children'].append(len(doc['nodes']))
        doc['nodes'].append({'name':f'rune.{team.lower()}.spin','translation':pivot.tolist(),'children':children,
                             'extras':{'rm':{'id':f'rune.{team.lower()}.spin'}}})
        selected.append({'id':f'rune.{team.lower()}.spin','axis':axis.tolist(),'pivot_m':pivot.tolist(),'bodies':names,'limits':None})
    # Check all expanded mesh-node rest transforms, before writing either asset.
    def frames(d):
        result={}
        def visit(i,parent):
            n=d['nodes'][i];m=node_matrix(n)
            world=parent@m
            if 'mesh'in n:result[i]=world
            for c in n.get('children',[]):visit(c,world)
        for scene in d['scenes']:
            for i in scene['nodes']:visit(i,np.eye(4))
        return result
    a,b=frames(before),frames(doc)
    if a.keys()!=b.keys() or any(np.max(np.abs(a[i]-b[i]))>1e-12 for i in a):raise ValueError('rest geometry moved')
    return selected

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--package',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    if args.out.exists():parser.error('output exists')
    manifest=json.loads((args.package/'manifest.json').read_text());binding=json.loads((args.package/'rmuc2027-bindings.json').read_text())
    doc,binary=read(args.package/'rune.glb'); fits=[]
    # Aperture fitting uses the exporter's baked bank coordinates. Reject a new
    # input convention instead of silently fitting in an incorrect local frame.
    if any(not np.allclose(node_matrix(n),np.eye(4),atol=1e-12,rtol=0) for n in doc['nodes']):
        raise ValueError('aperture fitting requires baked identity source transforms')
    for bank in ['bank-a','bank-b']:
        arm_names=[f'rune/banks/{bank}/arm-{i}' for i in range(5)]
        fit=[aperture(doc,binary,next(i for i,n in enumerate(doc['nodes']) if n.get('name')==name)) for name in arm_names]
        centers=np.array([f[1] for f in fit]);pivot=centers.mean(0);normal=fit[0][2]
        radii=np.linalg.norm(centers-pivot,axis=1)
        if np.max(np.abs(radii-.7))>.005:raise ValueError(f'non-native orbit {radii}')
        fits.append((bank,arm_names,fit,pivot,normal))
    axis=fits[1][3]-fits[0][3];axis/=np.linalg.norm(axis)
    root=manifest['assets']['rune']['placements_in_source_arena_frame'][0];rotation=Rotation.from_quat(root['rotation_xyzw']);translation=np.array(root['translation_m'])-[0,1.624343675041,manifest['floor_top_source_z_m']]
    geometries={};banks=[];report=[]
    for i,(bank,arms,fit,pivot,normal) in enumerate(fits):
        outward=axis*(1 if i else -1);team='Red' if rotation.apply(outward)[0]>0 else 'Blue'
        targets=[]
        for _,center,_,radius,error,node,gap in fit:
            v=center-pivot;v/=np.linalg.norm(v);u=np.cross(v,outward);u/=np.linalg.norm(u);v=np.cross(outward,u)
            frame=rotation.as_matrix()@np.column_stack([outward,u,v]);q=Rotation.from_matrix(frame).as_quat()
            targets.append({'translation_m':(rotation.apply(center+outward*.004)+translation).tolist(),'rotation_wxyz':[float(q[3]),*q[:3].tolist()]})
            report.append({'bank':bank,'source_node':node,'aperture_radius_m':float(radius),'radial_fit_error_m':float(error),'max_angular_gap_rad':gap})
        geometries[team]={'pivot_m':(rotation.apply(pivot)+translation).tolist(),'axis':rotation.apply(axis).tolist(),'targets':targets,'arm_nodes':arms}
        banks.append((bank,pivot,axis,team,arms))
    visual_motion=bind(doc,banks);collision,collision_binary=read(args.package/'rune-collision.glb');motions=bind(collision,banks)
    args.out.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(args.package,args.out)
    write(args.out/'rune.glb',doc,binary);write(args.out/'rune-collision.glb',collision,collision_binary)
    binding['rune_motion']=sorted(motions,key=lambda m:0 if '.red.'in m['id'] else 1)
    binding['world']['runes']=[geometries[t] for t in ['Red','Blue']]
    binding['rune_arms']=[next(arms for b,p,a,team,arms in banks if team==t) for t in ['Red','Blue']]
    binding['rune_fit']={'producer_sha256':sha(Path(__file__)),'input_visual_sha256':sha(args.package/'rune.glb'),
                         'input_collision_sha256':sha(args.package/'rune-collision.glb'),'detectors':report,
                         'geometry_buffers_exact':True,'rest_transform_max_error_m':1e-12,'exposure_m':.004}
    (args.out/'rmuc2027-bindings.json').write_text(json.dumps(binding,indent=2)+'\n')
    asset=manifest['assets']['rune'];asset['visual_sha256']=sha(args.out/'rune.glb');asset['collision_sha256']=sha(args.out/'rune-collision.glb');manifest['rmuc2027_bindings']['sha256']=sha(args.out/'rmuc2027-bindings.json')
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'banks':[{ 'bank':b,'team':t,'pivot_m':p.tolist()}for b,p,a,t,arms in banks], 'visual_moving_parts':[len(m['bodies'])for m in visual_motion], 'collision_moving_parts':[len(m['bodies'])for m in motions], 'detectors':report},indent=2))
if __name__=='__main__':main()
