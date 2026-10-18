#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check T through the app/host path with a paused source-bound Hero or stair donor.

Start rm-simulator with --console --window-mode headless --start-paused
--third-person --cad-assets field --robot-assets PACKAGE, then run this script.
"""
import argparse
import json
from pathlib import Path
import socket
from console import ConsoleClient


def run(client, directory):
    state=client.call({'type':'ready'})
    client.call({'type':'pause','paused':True})
    client.call({'type':'capture','captured':True})
    c=state['chassis']['config']
    stairs=c.get('climbing')
    folding=c.get('folding')
    serial=c.get('articulation')
    if not (stairs or folding or (serial and serial['limit_rad']>0)):raise RuntimeError('requires a source-bound transformable robot')
    limit=serial['limit_rad'] if serial else (stairs['deployed_angle_rad']-stairs['travel_angle_rad'] if stairs else folding['folded_angle_rad'])
    assert abs(state['chassis']['fold_angle_rad'])<1e-6, 'start at travel/unfolded endpoint'
    assert state['team_robot']['ready'], state['team_robot']
    directory.mkdir(parents=True,exist_ok=True)
    def shot(name):client.call({'type':'screenshot','path':str((directory/(name+'.png')).resolve())})
    def step(ticks):return client.call({'type':'step','ticks':ticks})
    def toggle():
        client.call({'type':'key','key':'t','pressed':True})
        step(1)
        client.call({'type':'key','key':'t','pressed':False})
    step(128);shot('travel')
    toggle();middle=step(64)
    angle=middle['chassis']['fold_angle_rad']
    assert 0.1<angle<limit, angle
    shot('mid-transform')
    deployed=step(512)
    assert abs(deployed['chassis']['fold_angle_rad']-limit)<1e-4, deployed['chassis']['fold_angle_rad']
    assert deployed['chassis']['command']['folded'] is True
    shot('deployed')
    toggle();returned=step(512)
    assert abs(returned['chassis']['fold_angle_rad'])<1e-4
    shot('returned')
    result={'profile':serial['profile'] if serial else ('stairs' if stairs else 'hero'),'mid_angle_rad':angle,'endpoint_rad':limit,
            'returned_rad':returned['chassis']['fold_angle_rad'],'team_robot':returned['team_robot']}
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=7790)
    parser.add_argument('--out',type=Path,default=Path('local-assets/mechanism-console'))
    args=parser.parse_args()
    with socket.create_connection(('127.0.0.1',args.port),130) as connection:
        with connection.makefile('rwb') as stream:
            print(json.dumps(run(ConsoleClient(stream,emit=lambda _:None),args.out),indent=2))


if __name__=='__main__':main()
