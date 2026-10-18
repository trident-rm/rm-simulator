#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Render the mechanism runner's recorded poses against its two-step fixture.

Uses the source-bound package and rm-map-tools' diagnostic rasterizer. This is
an offline CAD inspection render, not an in-game screen recording. Physics body
poses, wheel centres, spin and actuator inputs come directly from the runner.
"""
import argparse
import csv
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation


def box(center, half):
    p=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)*half+center
    t=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return p[t]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools',type=Path,default=Path('../rm-map-tools'))
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.tools.resolve()/'python'))
    from gltf_scene import read_glb,mesh_instances
    from stair_linkage import frames,C
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(args.motion.open())]
    document,binary=read_glb(args.package/'neutral.glb');groups={};rest=frames(0.)
    for ni,pi,p,t,m in mesh_instances(document,binary):
        name=document['nodes'][ni]['extras']['motion_group'];frame=rest[name]
        local=(p-frame[:3,3])@frame[:3,:3]
        linear=np.array(document['materials'][m]['pbrMetallicRoughness']['baseColorFactor'][:3])
        rgb=np.where(linear<=.0031308,linear*12.92,1.055*linear**(1/2.4)-.055)
        groups.setdefault(name,[]).append((local[t],np.tile(rgb,(len(t),1))))
    # Simple physical armor housings and emissive-strip colors for this diffuse render.
    armor=json.loads((args.package/'manifest.json').read_text())['armor']['modules']
    for m in armor:
        p=m['body_face_pose_flu'];w,x,y,z=p['rotation_wxyz'];r=Rotation.from_quat([x,y,z,w]).as_matrix()
        for center,half,color in [([-0.0095,0,0],[0.0095,.0705,.0675],[.075,.08,.09]),
                                  ([.0002,-.065,0],[.001,.004,.028],[1,.12,.08]),
                                  ([.0002,.065,0],[.001,.004,.028],[1,.12,.08])]:
            tri=(box(center,half)@r.T+np.array(p['translation_m']))@C
            groups['body'].append((tri,np.tile(color,(len(tri),1))))
    groups={k:(np.concatenate([v[0] for v in values]),np.concatenate([v[1] for v in values])) for k,values in groups.items()}
    # Identical extents to the physics runner (FLU), mapped into native glTF axes.
    terrain=np.concatenate([box([.8,0,.1],[.8,1.5,.1]),box([2.4,0,.2],[.8,1.5,.2]),box([.8,0,-.025],[2.1,1.8,.025])])@C
    terrain_colors=np.repeat([[.42,.46,.50]],len(terrain),axis=0)
    width,height=1280,720
    forward=np.array([1.,.22,.10]);forward/=np.linalg.norm(forward)
    right=np.cross([0,1,0],forward);right/=np.linalg.norm(right);up=np.cross(forward,right)
    camera=np.column_stack([right,up,forward]);light=forward+.8*up-.3*right;light/=np.linalg.norm(light)
    # Fixed zoom, following the body's recorded translation; no motion smoothing.
    scale=340.
    try:
        font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',23)
        small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17)
    except OSError:
        font=ImageFont.load_default(size=23);small=ImageFont.load_default(size=17)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rm-stair-video-') as temp:
        temp=Path(temp);helper=temp/'raster'
        subprocess.run(['clang++','-O3',str(args.tools/'benchmarks/2026-10-17-base-analysis/visibility-raster.cpp'),'-o',str(helper)],check=True)
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r','32','-i','-',
                                  '-an','-c:v','libx264','-preset','medium','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)],stdin=subprocess.PIPE)
        for index,row in enumerate(rows):
            w,x,y,z=[row[k] for k in ['qw','qx','qy','qz']];body=C.T@Rotation.from_quat([x,y,z,w]).as_matrix()@C
            origin=C.T@np.array([row['x_m'],row['y_m'],row['z_m']]);posed=frames(row['input_rad'])
            triangles=[terrain];colors=[terrain_colors]
            for name,(tri,rgb) in groups.items():
                if name.startswith('wheel-'):
                    i=int(name[-1]);pos=C.T@np.array([row[f'wheel_{i}_{axis}'] for axis in ['x','y','z']])
                    rotation=body@Rotation.from_rotvec([-row[f'wheel_{i}_spin'],0,0]).as_matrix()
                else:
                    frame=posed[name];rotation=body@frame[:3,:3];pos=origin+body@frame[:3,3]
                triangles.append(tri@rotation.T+pos);colors.append(rgb)
            geometry=np.concatenate(triangles);rgb=np.concatenate(colors)
            normals=np.cross(geometry[:,1]-geometry[:,0],geometry[:,2]-geometry[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-20)
            shade=np.clip(rgb*(.55+.45*abs(normals@light))[:,None]*255,0,255).astype('uint8')
            center=(C.T@np.array([row['x_m']+.35,row['y_m'],row['z_m']+.35]))@camera
            projected=geometry@camera-center;projected[:,:,0]=projected[:,:,0]*scale+width/2;projected[:,:,1]=-projected[:,:,1]*scale+height/2+12
            records=np.empty(len(geometry),dtype=[('p','<f4',(3,3)),('c','u1',(3,))]);records['p']=projected;records['c']=shade
            (temp/'input.bin').write_bytes(struct.pack('<iii',width,height,len(geometry))+records.tobytes())
            subprocess.run([str(helper),str(temp/'input.bin'),str(temp/'frame.ppm'),str(temp/'owners.bin')],check=True)
            image=Image.open(temp/'frame.ppm');draw=ImageDraw.Draw(image)
            draw.rectangle([0,0,width,82],fill=(24,32,42));draw.text((24,15),'DreamChaser stair Heavy | two 200 mm steps',font=font,fill='white')
            mode='RAISE FRONT / EXTEND REAR' if row['input_rad']>.1 else 'TRAVEL / RETURN LINKAGE'
            draw.text((24,49),f"{row['tick']/128:5.2f} s   |   {mode}   |   input {row['input_rad']*180/np.pi:4.1f} deg",font=small,fill=(185,213,229))
            draw.rectangle([0,height-35,width,height],fill=(24,32,42));draw.text((24,height-27),'Recorded 128 Hz physics; source CAD inspection render. No body pose injection.',font=small,fill=(205,215,225))
            encoder.stdin.write(image.tobytes())
            if index in [0,len(rows)//2,len(rows)-1]:image.save(args.out.with_name(args.out.stem+f'-{index:04d}.png'))
            if index%32==0:print(f'{index}/{len(rows)} frames',flush=True)
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('ffmpeg failed')
    print(args.out)


if __name__=='__main__':main()
