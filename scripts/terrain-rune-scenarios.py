#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise native 2027 terrain and Rune through physical cards/projectiles.

Requires a paused embedded referee on a Rune-bound 2027 field. Resets the match,
creates one training bot, and leaves paused Idle on success; failures stay inspectable.
No RFID flags, Rune progress, ring scores or benefits are injected.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import socket
import sys
from console import ConsoleClient, assert_state

spec=importlib.util.spec_from_file_location('field_scenarios',Path(__file__).with_name('control-field-scenarios.py'))
field=importlib.util.module_from_spec(spec);spec.loader.exec_module(field)

def qmul(a,b):
    w,x,y,z=a;v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]
def rotate(q,p):
    return qmul(qmul(q,[0,*p]),[q[0],-q[1],-q[2],-q[3]])[1:]
def interior(zone):
    polygon=zone['polygon_m'];center=[sum(p[i] for p in polygon)/len(polygon) for i in [0,1]]
    if field.inside(center,polygon):return [*center,zone['floor_m']+.4]
    lo=[min(p[i]for p in polygon)for i in [0,1]];hi=[max(p[i]for p in polygon)for i in [0,1]]
    candidates=[[lo[0]+(hi[0]-lo[0])*x/30,lo[1]+(hi[1]-lo[1])*y/30]for x in range(1,30)for y in range(1,30)]
    p=min((p for p in candidates if field.inside(p,polygon)),key=lambda p:math.dist(p,center))
    return [*p,zone['floor_m']+.4]
def angle(r,now):
    t=(now-r['motion_epoch_ns'])*1e-9
    if r['stage']=='Big' and r['phase'] in ['Activating','Failed']:
        a=r['amplitude_urad_s']*1e-6;w=r['omega_urad_s']*1e-6;delta=a/w*(1-math.cos(w*t))+(2.090-a)*t
    else:delta=math.pi/3*t
    return r['angle_epoch_nrad']*1e-9+r['direction']*delta

def run(client, only="both"):
    state=client.call({'type':'ready'})
    if state.get('role')!='Referee' or len(state.get('rmuc_2027_config',{}).get('runes',[]))!=2:
        raise RuntimeError('requires embedded --referee and a native Rune-bound 2027 package')
    def call(command):return client.call(command)
    def world(command):return call({'type':'world','command':command})
    def step(ticks):return call({'type':'step','ticks':ticks})
    def state_now():return call({'type':'state'})
    def check(expected):
        s=state_now();assert_state(s,expected);return s
    call({'type':'pause','paused':True});world('ClearBots');world({'Referee':'ResetMatch'})
    before={r['id']for r in state_now()['rmuc_2027']['robots']}
    state=world({'SpawnBot':{'team':'Red','spin_rad_s':0}})
    bot=next(r['id']for r in state['rmuc_2027']['robots'] if r['id']not in before)
    config=state['rmuc_2027_config'];cards=config['terrain_zones']
    def card(course,pad,index=0):return next(c for c in cards if c['card']=={'side':'Red','course':course,'index':index,'pad':pad})
    def place(position):return world({'PlaceBot':{'chassis':bot,'position_m':position,'yaw_deg':0}})
    def touch(c):place(interior(c['zone']));return step(1)
    def start():world({'Referee':'ResetMatch'});world({'Referee':'StartMatch'});step(640)
    if only != 'rune':
        start()
        touch(card('Road',0));touch(card('Road',1))
        state=check({'/rmuc_2027/robots/0/terrain/defense_pct':25,'/rmuc_2027/robots/0/terrain/experience_awarded/1':True})
        ready=state['rmuc_2027']['robots'][0]['terrain']['road_ready_ns']
        touch(card('Road',0));touch(card('Road',1))
        check({'/rmuc_2027/robots/0/terrain/road_ready_ns':ready})
        for pad in [0,1,2]:touch(card('Tunnel',pad))
        check({'/rmuc_2027/robots/0/terrain/defense_pct':50,'/rmuc_2027/robots/0/terrain/experience_awarded/2':True})
        for course in [0,1]:
            touch(card('Highland',0,course));touch(card('Highland',1,course))
            check({'/rmuc_2027/robots/0/terrain/experience_awarded/0':True})
        # An unrelated Base card cancels a pending Road attempt.
        touch(card('Road',0))
        base=next(z for z in config['zones']if z['side']=='Red'and z['point']=='Base')
        place(field.positions(base)[0]);step(1)
        check({'/rmuc_2027/robots/0/terrain/progress':None})
        world({'Referee':{'SetRobotHp':{'robot':bot,'hp':0}}})
        check({'/rmuc_2027/robots/0/terrain/until_ns':[0,0,0,0]})
    if only == 'terrain':
        world('ClearBots');world({'Referee':'ResetMatch'});print(json.dumps({'scenario':'terrain-2027','ok':True}),flush=True);return
    world('ClearBots');start()
    def shoot(team,blade):
        s=state_now();g=s['rmuc_2027'];r=g['runes'][team];geometry=config['runes'][team]
        phase=angle(r,g['control']['state']['elapsed_ns']);axis=geometry['axis'];q=[math.cos(phase/2),*[v*math.sin(phase/2)for v in axis]]
        rest=geometry['targets'][blade];p=rotate(q,[a-b for a,b in zip(rest['translation_m'],geometry['pivot_m'])]);p=[a+b for a,b in zip(p,geometry['pivot_m'])]
        orientation=qmul(q,rest['rotation_wxyz']);normal=rotate(orientation,[1,0,0]);muzzle=[a+.2*b for a,b in zip(p,normal)]
        world({'SpawnProjectile':{'muzzle':{'translation_m':muzzle,'rotation_wxyz':qmul(orientation,[0,0,0,1])},'shot':{'caliber':'Mm17','speed_m_s':25}}})
        return step(8)
    for team,name in enumerate(['Red','Blue']):
        world({'Referee':{'ActivateRune':{'team':name}}})
        for hit in range(5):
            r=state_now()['rmuc_2027']['runes'][team];blade=r['targets'].index(True)
            s=shoot(team,blade)
            assert_state(s,{f'/rmuc_2027/runes/{team}/groups':hit+1})
        check({f'/rmuc_2027/runes/{team}/phase':'Activated',f'/rmuc_2027/runes/{team}/buff/defense_pct':25})
    # Big stage and optional second hits: ten physical center strikes -> highest table row.
    step(180*128)
    world({'Referee':{'ActivateRune':{'team':'Red'}}})
    for _ in range(10):
        r=state_now()['rmuc_2027']['runes'][0];shoot(0,r['targets'].index(True))
    check({'/rmuc_2027/runes/0/phase':'Activated','/rmuc_2027/runes/0/big_successes':1,
           '/rmuc_2027/runes/0/buff/attack_pct':300,'/rmuc_2027/runes/0/buff/cooling':5})
    world({'Referee':'ResetMatch'})
    print(json.dumps({'scenario':'terrain-rune-2027','ok':True,'terrain_cards':len(cards)}),flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=7790);parser.add_argument('--only',choices=['both','terrain','rune'],default='both');args=parser.parse_args()
    with socket.create_connection(('127.0.0.1',args.port),130)as connection:
        with connection.makefile('rwb')as stream:run(ConsoleClient(stream,emit=lambda s:print(s,flush=True)),args.only)
if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError,StopIteration)as error:print(f'terrain-rune-scenarios: {error}',file=sys.stderr);sys.exit(1)
