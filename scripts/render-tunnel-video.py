#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Render recorded tunnel physics with source shells and a cutaway roof.

Record using robot_tunnel chassis.json stowed .25 motion.jsonl. Source joint
frames are evaluated by Rust, not approximated from the trajectory here.
Requires rm-map-tools' Python environment, its rasterizer, clang++ and FFmpeg.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation
import importlib.util
_spec=importlib.util.spec_from_file_location('stair_video',Path(__file__).with_name('render-stair-video.py'))
_stair=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_stair)
box=_stair.box

C=np.array([[0,0,1],[1,0,0],[0,1,0.]])


def matrix(p):
    w,x,y,z=p['rotation_wxyz'];r=np.eye(4);r[:3,:3]=C.T@Rotation.from_quat([x,y,z,w]).as_matrix()@C;r[:3,3]=C.T@np.array(p['translation_m']);return r


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools',type=Path,default=Path('../rm-map-tools'))
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--title',required=True)
    args=parser.parse_args();sys.path.insert(0,str(args.tools.resolve()/'python'))
    from gltf_scene import read_glb,mesh_instances
    records=[json.loads(s) for s in args.motion.read_text().splitlines()];header=records[0];rows=records[1:];cfg=header['config'];roof=header['roof_m']
    if len(rows)<2:raise ValueError('recording has no motion')
    fps=128/(rows[1]['tick']-rows[0]['tick']);assert fps==32
    articulation=cfg.get('articulation');folding=cfg.get('folding')
    limit=articulation['limit_rad'] if articulation else folding['folded_angle_rad']
    frames={name:matrix(p) for name,p in header['reference_frames']}
    for i,hub in enumerate(cfg['wheel_hubs_m']):
        yaw=0. if cfg['mecanum'] else np.arctan2(hub[1],hub[0])-np.pi/2
        f=np.eye(4);f[:3,:3]=C.T@Rotation.from_rotvec([0,0,yaw]).as_matrix()@C
        # Neutral GLB tyres have the source loaded centre, not the unloaded ray anchor.
        # Fitted source packages carry their exact wheel frames directly.
        frames[f'wheel-{i}']=f
    d,b=read_glb(args.package/'neutral.glb');manifest=json.loads((args.package/'manifest.json').read_text());groups={}
    from gltf_scene import scene_nodes
    worlds={ni:m for ni,m,_ in scene_nodes(d)}
    for ni,m,_ in scene_nodes(d):
        name=d['nodes'][ni].get('name','')
        if name.startswith('link/wheel-'):
            # Use the exported reference wheel frame, preserving its orientation.
            frames[name[5:]]=m.copy()
    body_offset=float(manifest.get('body_origin_y_m',0.))
    if cfg.get('folding'):
        for name in list(frames):
            if name.startswith('wheel-'):frames[name][1,3]-=body_offset
    for ni,pi,p,t,mi in mesh_instances(d,b):
        name=d['nodes'][ni]['extras']['motion_group'];frame=frames[name]
        local=(p-[0,body_offset,0]-frame[:3,3])@frame[:3,:3]
        linear=np.array(d['materials'][mi]['pbrMetallicRoughness']['baseColorFactor'][:3]);rgb=np.where(linear<=.0031308,linear*12.92,1.055*linear**(1/2.4)-.055)
        groups.setdefault(name,[]).append((local[t],np.tile(rgb,(len(t),1))))
    body_group='source-body' if cfg.get('articulation') else 'body'
    for module in manifest['armor']['modules']:
        p=module['body_face_pose_flu'];w,x,y,z=p['rotation_wxyz'];r=Rotation.from_quat([x,y,z,w]).as_matrix();half=module['housing_half_m'];span=module['light_span_m']/2;length=module['light_length_m']/2
        for center,size,color in [([-half[0],0,0],half,[.075,.08,.09]),([.0002,-span,0],[.001,.004,length],[1,.12,.08]),([.0002,span,0],[.001,.004,length],[1,.12,.08])]:
            tri=(box(center,size)@r.T+np.array(p['translation_m']))@C
            groups[body_group].append((tri,np.tile(color,(len(tri),1))))
    groups={k:(np.concatenate([v[0] for v in values]),np.concatenate([v[1] for v in values])) for k,values in groups.items()}
    # The physical slab is x=0..2, y=-1..1. Retain its far strip for a cutaway.
    terrain=np.concatenate([box([1,-.775,roof+.05],[1,.225,.05]),box([1,0,-.02],[3.8,1.2,.02])])@C
    terrain_colors=np.concatenate([np.tile([.30,.48,.59],(12,1)),np.tile([.32,.35,.39],(12,1))])
    width,height=1280,720;scale=850.
    forward=np.array([1.,.015,.025]);forward/=np.linalg.norm(forward);right=np.cross([0,1,0],forward);right/=np.linalg.norm(right);up=np.cross(forward,right)
    camera=np.column_stack([right,up,forward]);light=forward+.8*up-.3*right;light/=np.linalg.norm(light)
    try:font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',23);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17)
    except OSError:font=ImageFont.load_default(size=23);small=ImageFont.load_default(size=17)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rm-tunnel-video-') as folder:
        folder=Path(folder);helper=folder/'raster';subprocess.run(['clang++','-O3',str(args.tools/'benchmarks/2026-10-17-base-analysis/visibility-raster.cpp'),'-o',str(helper)],check=True)
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(fps),'-i','-',
                                  '-an','-c:v','libx264','-preset','medium','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)],stdin=subprocess.PIPE)
        for index,row in enumerate(rows):
            body=matrix(row['pose']);posed={name:matrix(p) for name,p in row['source_frames']};triangles=[terrain];colors=[terrain_colors]
            for name,(tri,rgb) in groups.items():
                if name.startswith('wheel-'):
                    i=int(name[-1]);wheel=row['wheels'][i];yaw=0. if cfg['mecanum'] else np.arctan2(cfg['wheel_hubs_m'][i][1],cfg['wheel_hubs_m'][i][0])-np.pi/2
                    rotation=body[:3,:3]@C.T@(Rotation.from_rotvec([0,0,yaw]).as_matrix()@Rotation.from_rotvec([0,-wheel['spin_rad'],0]).as_matrix())@C
                    pos=C.T@np.array(wheel['hub_m'])
                else:frame=body@posed[name];rotation=frame[:3,:3];pos=frame[:3,3]
                triangles.append(tri@rotation.T+pos);colors.append(rgb)
            geometry=np.concatenate(triangles);rgb=np.concatenate(colors);normal=np.cross(geometry[:,1]-geometry[:,0],geometry[:,2]-geometry[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-20)
            shade=np.clip(rgb*(.55+.45*abs(normal@light))[:,None]*255,0,255).astype('uint8')
            center=(C.T@np.array([row['pose']['translation_m'][0],0,.20]))@camera
            projected=geometry@camera-center;projected[:,:,0]=projected[:,:,0]*scale+width/2;projected[:,:,1]=-projected[:,:,1]*scale+height/2+45
            data=np.empty(len(geometry),dtype=[('p','<f4',(3,3)),('c','u1',(3,))]);data['p']=projected;data['c']=shade
            (folder/'input.bin').write_bytes(struct.pack('<iii',width,height,len(geometry))+data.tobytes());subprocess.run([str(helper),str(folder/'input.bin'),str(folder/'frame.ppm'),str(folder/'owners.bin')],check=True)
            image=Image.open(folder/'frame.ppm');draw=ImageDraw.Draw(image)
            # Centre-plane guide removes the apparent clearance from the far roof's depth offset.
            def screen(point):
                v=(C.T@np.array(point))@camera-center
                return (v[0]*scale+width/2,-v[1]*scale+height/2+45)
            a=np.array(screen([0,0,roof]));b=np.array(screen([2,0,roof]));length=np.linalg.norm(b-a)
            for distance in np.arange(0,length,18):
                start=a+(b-a)*(distance/length);end=a+(b-a)*(min(distance+10,length)/length)
                draw.line([tuple(start),tuple(end)],fill=(116,211,236),width=2)
            draw.rectangle([0,0,width,84],fill=(24,32,42));draw.text((24,15),args.title+' | 250 mm tunnel',font=font,fill='white')
            mode='FIXED HEIGHT / NEUTRAL GUN' if limit==0 else ('STOWED' if row['input_rad']>.99*limit else 'TRANSFORMING')
            draw.text((24,49),f"{row['tick']/128:5.2f} s  |  {mode}  |  x={row['pose']['translation_m'][0]:.2f} m",font=small,fill=(185,213,229))
            draw.rectangle([0,height-40,width,height],fill=(24,32,42));draw.text((24,height-30),'128 Hz contact physics | cutaway roof | cyan dashes: 250 mm at the centre plane',font=small,fill=(205,215,225))
            encoder.stdin.write(image.tobytes())
            if index in [0,96,225,len(rows)-1]:image.save(args.out.with_name(args.out.stem+f'-{index:04d}.png'))
            if index%64==0:print(f'{index}/{len(rows)} frames',flush=True)
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('FFmpeg failed')
    print(args.out)


if __name__=='__main__':main()
