#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Record a scanning Sentry attacking through ordinary host Fire commands.

The host controller supplies aim and chassis sweep; this recording harness acts
as the trigger operator. No damage/pose playback is injected. Optional decoded
information is a referee receiver proxy for the HUD, not a production decoder.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from console import ConsoleClient


def record(c, args, folder, audit):
    from PIL import Image, ImageDraw, ImageFont
    state=c.call({'type':'ready'})
    c.call({'type':'pause','paused':True})
    c.call({'type':'release_inputs'})
    try:c.call({'type':'spawn','position_m':[4.7,3.5,1.],'yaw_deg':-90.})
    except RuntimeError as error:
        if 'another life' not in str(error):raise
    c.call({'type':'release_inputs'})
    state=c.call({'type':'state'})
    own=state['chassis_id']
    assert own is not None
    for position in [[6.7,3.5,1.],[6.2,3.5,1.],[3.5,3.5,1.]]:
        try:
            state=c.call({'type':'world','command':{'SpawnSentry':{'team':'Blue','position_m':position,'yaw_deg':180.}}})
            break
        except RuntimeError as error:
            if 'obstructed' not in str(error):raise
    else:raise RuntimeError('demo locations are obstructed')
    enemy=next(b['id'] for b in state['chassis_all'] if b['id']!=own)
    c.call({'type':'world','command':{'Referee':'StartMatch'}})
    state=c.call({'type':'step','ticks':640})
    c.call({'type':'capture','captured':True})
    c.call({'type':'world','command':{'SetSentryAutoAim':{'chassis':own,'enabled':True}}})
    c.call({'type':'camera','third_person':True})
    start=state['tick'];last_fire=start-32;fire_count=0;encoder=None
    screenshot=folder/'frame.png'
    font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',22)
    small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17)
    audit.update(own=own,enemy=enemy,initial_hp=400,trigger='scripted operator via Command::Fire',frames=[])

    def robot():return next(r for r in state['rmuc_2027']['robots'] if r['id']==enemy)
    def intel():
        nonlocal state
        r=robot()
        info={'hp':r['hp'],'max_hp':r['max_hp'],'allowance':r['combat']['allowance']}
        state=c.call({'type':'world','command':{'Referee':{'RadarInformation':{'team':'Red','robot':enemy,'information':info}}}})

    def capture(label,repeat=1):
        nonlocal encoder
        c.call({'type':'screenshot','path':str(screenshot)})
        with Image.open(screenshot) as source:image=source.convert('RGB')
        draw=ImageDraw.Draw(image);w,h=image.size
        draw.rectangle([0,h-84,w,h],fill=(20,27,36))
        draw.text((20,h-76),f'Sentry attack | {label}',font=font,fill='white')
        draw.text((20,h-46),f'{(state["tick"]-start)/128:4.1f} s | enemy HP {robot()["hp"]}/400 | '
                  f'{fire_count} trigger requests | native damage {state["rmuc_2027"]["attack_damage"][0]}',font=small,fill=(117,224,166))
        draw.text((20,h-23),'Host aim + chassis sweep | scripted trigger | real projectiles | 1x playback',font=small,fill=(184,210,227))
        if encoder is None:
            encoder=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pixel_format','rgb24',
                '-video_size',f'{w}x{h}','-framerate',str(args.fps),'-i','-','-an','-c:v','libx264',
                '-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)],stdin=subprocess.PIPE)
        pixels=image.tobytes()
        for _ in range(repeat):encoder.stdin.write(pixels)

    defeated_tick=None
    try:
        intel();capture('Searching',round(args.fps*.5))
        for frame in range(round(args.seconds*args.fps)):
            ticks=round((frame+1)*128/args.fps)-round(frame*128/args.fps)
            state=c.call({'type':'step','ticks':ticks})
            view=next(v for v in state['sentry_tracking'] if v['chassis']==own)
            body=state['chassis']
            yaw_error=math.atan2(math.sin(body['command']['aim_yaw_rad']-body['held_aim_rad'][0]),
                                 math.cos(body['command']['aim_yaw_rad']-body['held_aim_rad'][0]))
            pitch_error=body['command']['aim_pitch_rad']-body['held_aim_rad'][1]
            ready=(view['target']==enemy and abs(yaw_error)<.018 and abs(pitch_error)<.018
                   and robot()['hp']>0 and state['tick']-last_fire>=32)
            if ready:
                try:
                    state=c.call({'type':'world','command':{'Fire':{'shooter':own}}})
                    fire_count+=1;last_fire=state['tick']
                except RuntimeError as error:
                    if not any(s in str(error) for s in ['cadence','heat','launcher','allowance']):raise
            if frame%max(1,args.fps//4)==0:intel()
            hp=robot()['hp'];label='Tracking and firing' if view['target']==enemy and hp else 'Searching'
            if hp==0:
                defeated_tick=defeated_tick or state['tick'];label='Target defeated — resuming sweep'
            audit['frames'].append({'tick':state['tick'],'hp':hp,'target':view['target'],
                'held_aim_rad':body['held_aim_rad'],'command':body['command'],'trigger_requests':fire_count,
                'damage':state['rmuc_2027']['attack_damage'][0]})
            capture(label)
            if defeated_tick and state['tick']-defeated_tick>=128:break
        audit.update(final_hp=robot()['hp'],fire_requests=fire_count,
                     damage=state['rmuc_2027']['attack_damage'][0],simulation_s=(state['tick']-start)/128)
        assert fire_count>0,'no aligned trigger requests'
        assert audit['damage']>0,'shots did not damage the opponent'
        if not args.allow_survivor:assert audit['final_hp']==0,'target was not defeated within recording deadline'
    finally:
        if encoder:
            encoder.stdin.close()
            if encoder.wait(timeout=60)!=0:raise RuntimeError('FFmpeg failed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app',type=Path,default=Path('target/nix/debug/rm-simulator'))
    p.add_argument('--out',type=Path,default=Path('local-assets/sentry-attack.mp4'))
    p.add_argument('--fps',type=int,choices=[16,32],default=32)
    p.add_argument('--seconds',type=float,default=12.)
    p.add_argument('--allow-survivor',action='store_true')
    args=p.parse_args();args.out=args.out.resolve();args.out.parent.mkdir(parents=True,exist_ok=True)
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    audit={}
    with args.out.with_suffix('.app.log').open('w') as log,tempfile.TemporaryDirectory() as temp:
        app=subprocess.Popen([str(args.app.resolve()),'--play','--third-person','--robot','sentry','--chassis','tunnel-omni',
            '--cad-assets',str(Path('field-2027').resolve()),'--robot-assets',str(Path('local-assets/robot-cad-survey/bound-tunnel-sentry').resolve()),
            '--console',f'127.0.0.1:{port}','--start-paused','--window-mode','headless'],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+90
            while True:
                try:connection=socket.create_connection(('127.0.0.1',port),120);break
                except ConnectionRefusedError:
                    if app.poll() is not None or time.monotonic()>deadline:raise RuntimeError('app startup failed; see app.log') from None
                    time.sleep(.1)
            with connection,connection.makefile('rwb') as stream:
                c=ConsoleClient(stream,emit=lambda _:None)
                try:record(c,args,Path(temp),audit)
                finally:c.call({'type':'quit'})
        finally:
            args.out.with_suffix('.json').write_text(json.dumps(audit,indent=2)+'\n')
            try:app.wait(timeout=10)
            except subprocess.TimeoutExpired:app.terminate();app.wait(timeout=10)
    print(json.dumps({k:v for k,v in audit.items() if k!='frames'}))
    print(args.out)


if __name__=='__main__':main()
