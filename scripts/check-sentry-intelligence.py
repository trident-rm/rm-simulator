#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise native Radar, independent HP observations and Sentry optical aiming in game.

Uses one owned Red Sentry and one host Blue Sentry, ordinary protocol commands,
actual motor motion and a source CAD scene. Writes screenshots and a JSON audit.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import time
from console import ConsoleClient


def run(c, out):
    state=c.call({'type':'ready'})
    c.call({'type':'pause','paused':True})
    c.call({'type':'release_inputs'})
    try:c.call({'type':'spawn','position_m':[4.7,3.5,1.],'yaw_deg':-90.})
    except RuntimeError as error:
        # Placement can invalidate an already queued pilot frame after applying.
        if 'another life' not in str(error):raise
    c.call({'type':'release_inputs'})
    placed=c.call({'type':'state'})['chassis']['pose']['translation_m']
    assert abs(placed[0]-4.7)<.1 and abs(placed[1]-3.5)<.1, placed
    own=state['chassis_id']
    for position in [[6.7,3.5,1.],[6.2,3.5,1.],[3.5,3.5,1.]]:
        try:
            state=c.call({'type':'world','command':{'SpawnSentry':{'team':'Blue','position_m':position,'yaw_deg':180.}}})
            break
        except RuntimeError as error:
            if 'obstructed' not in str(error):raise
    else:raise RuntimeError('all demo locations are obstructed')
    enemy=next(b['id'] for b in state['chassis_all'] if b['id']!=own)
    c.call({'type':'world','command':{'Referee':'StartMatch'}})
    state=c.call({'type':'step','ticks':640})
    c.call({'type':'capture','captured':True})
    c.call({'type':'world','command':{'SetSentryAutoAim':{'chassis':own,'enabled':True}}})
    start=state['chassis']['pose']['rotation_wxyz']
    shots=state['shots_fired'] if 'shots_fired' in state else None
    samples=[]
    acquired=False
    for _ in range(18):
        state=c.call({'type':'step','ticks':26})
        position=next(b['pose']['translation_m'] for b in state['chassis_all'] if b['id']==enemy)
        state=c.call({'type':'world','command':{'Referee':{'RadarReport':{'team':'Red','robot':enemy,'position_m':position[:2]}}}})
        tracking=next(v for v in state['sentry_tracking'] if v['chassis']==own)
        acquired |= tracking['target']==enemy
        contact=next((d for d in state['enemy_contacts'] if d['robot']==enemy),None)
        if contact: assert contact['hp'] is None, 'position report leaked hidden health'
        samples.append({'tick':state['tick'],'tracking':tracking,'contacts':state['enemy_contacts']})
    assert acquired, 'no optical acquisition during the sweep'
    q=state['chassis']['pose']['rotation_wxyz']
    rotation=2*math.acos(min(1,abs(sum(a*b for a,b in zip(start,q)))))
    assert rotation>.3, ('chassis did not sweep',rotation)
    c.call({'type':'screenshot','path':str(out/'position-only.png')})
    c.call({'type':'world','command':{'Referee':{'SetRobotHp':{'robot':enemy,'hp':123}}}})
    info={'hp':123,'max_hp':400,'allowance':77,'defense_pct':25}
    state=c.call({'type':'world','command':{'Referee':{'RadarInformation':{'team':'Red','robot':enemy,'information':info}}}})
    contact=next(d for d in state['enemy_contacts'] if d['robot']==enemy)
    assert contact['hp']==123 and contact['allowance']==77
    c.call({'type':'screenshot','path':str(out/'decoded-information.png')})
    state=c.call({'type':'step','ticks':128})
    assert all(d['hp'] is None for d in state['enemy_contacts']), 'decoded information did not expire'
    c.call({'type':'world','command':{'SetSentryAutoAim':{'chassis':own,'enabled':False}}})
    state=c.call({'type':'step','ticks':8})
    assert not next(v for v in state['sentry_tracking'] if v['chassis']==own)['enabled']
    assert state.get('shots_fired')==shots, 'aim controller fired a projectile'
    result={'acquired':acquired,'rotation_rad':rotation,'samples':samples,'final_tick':state['tick'],'manual_restored':True,'initial_shots':shots}
    (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app',type=Path,default=Path('target/nix/debug/rm-simulator'))
    p.add_argument('--out',type=Path,default=Path('local-assets/sentry-intelligence'))
    args=p.parse_args();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    with (args.out/'app.log').open('w') as log:
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
                try: print(json.dumps({k:v for k,v in run(c,args.out).items() if k!='samples'}))
                finally:c.call({'type':'quit'})
        finally:
            try:app.wait(timeout=10)
            except subprocess.TimeoutExpired:app.terminate();app.wait(timeout=10)


if __name__=='__main__':main()
