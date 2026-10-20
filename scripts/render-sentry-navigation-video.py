#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Test and optionally record one continuous, host-planned three-tunnel tour.

Launches a fresh paused app, spawns one EGA Sentry once, and sends only ordinary
NavigateSentry goals. No chassis pose or joint playback is injected. --check-only
skips GPU captures/FFmpeg; both modes write a JSON audit and fail on an incomplete
crossing, failed goal or slow planning. Source robot assets remain local.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time

from console import ConsoleClient


TOUR = [
    ('Road tunnel', [4.5, 6.8]),
    ('Approach far-highland tunnel', [2.5, 6.93]),
    ('Far-highland tunnel', [-2.5, 6.93]),
    ('Near-highland tunnel', [-3.5, 5.7]),
]

# Navigation survey witnesses, not RFID/rule definitions. Sample the actual
# folded chassis under each source roof, then require the opposite portal.
PASSAGES = {
    'Road': ([4.7, 5.2], [0., 1.], .55, 0., (.08, .34)),
    'Near highland': ([0., 5.93], [1., 0.], 1.55, -.95, (-.03, .13)),
    'Far highland': ([0., 6.93], [1., 0.], 1.55, 0., (.12, .30)),
}


class Crossings:
    """Require entry, folded under-roof witness and opposite-side exit in order."""
    def __init__(self):
        self.pending = {}
        self.completed = {}
        self.witnessed = set()

    def observe(self, body, tick):
        x, y, z = body['pose']['translation_m']
        for name, (origin, axis, span, witness, height) in PASSAGES.items():
            if name in self.completed:
                continue
            dx, dy = x-origin[0], y-origin[1]
            along = dx*axis[0]+dy*axis[1]
            lateral = -dx*axis[1]+dy*axis[0]
            if abs(lateral) > .23:
                self.pending.pop(name, None)
                continue
            attempt = self.pending.setdefault(name, {'side': None, 'witness_tick': None})
            if abs(along) > span:
                side = 1 if along > 0 else -1
                if attempt['witness_tick'] is not None and side != attempt['side']:
                    self.completed[name] = {**attempt, 'exit_tick': tick}
                else:
                    attempt.update(side=side, witness_tick=None)
            elif (attempt['side'] is not None and abs(along-witness) < .18
                  and body['fold_angle_rad'] > .99 and height[0] < z < height[1]):
                attempt['witness_tick'] = tick
                self.witnessed.add(name)


class Movie:
    """Capture the live app; stream annotated RGB frames directly to FFmpeg."""
    def __init__(self, client, args, temporary):
        from PIL import Image, ImageDraw, ImageFont
        self.Image, self.ImageDraw = Image, ImageDraw
        self.client, self.args = client, args
        self.screenshot = temporary / 'frame.png'
        self.encoder = None
        self.frames = 0
        self.saved = set()
        try:
            self.font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 22)
            self.small = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 17)
        except OSError:
            self.font = ImageFont.load_default(size=22)
            self.small = ImageFont.load_default(size=17)

    def add(self, state, body, label, crossings, elapsed_s, plan_s, duration):
        p = body['pose']['translation_m']
        # Look from the arena interior, away from the perimeter wall. Keep
        # native path overlays and actual source scenery visible throughout.
        self.client.call({'type': 'camera', 'position_m': [p[0]+3., p[1]-3., p[2]+1.8],
                          'yaw_deg': 135., 'pitch_deg': -22.4})
        self.client.call({'type': 'screenshot', 'path': str(self.screenshot)})
        with self.Image.open(self.screenshot) as source:
            image = source.convert('RGB')
        draw = self.ImageDraw.Draw(image)
        w, h = image.size
        draw.rectangle([0, h-86, w, h], fill=(20, 27, 36))
        draw.text((20, h-77), f'Three-tunnel tour | {label}', font=self.font, fill='white')
        nav = state['navigation'][0]
        detail = (f'{elapsed_s:5.1f} s simulation | {nav["message"]} | '
                  f'last goal-to-path: {plan_s*1000:.0f} ms')
        draw.text((20, h-47), detail, font=self.small, fill=(184, 210, 227))
        draw.text((20, h-24), f'Crossed: {", ".join(crossings.completed) or "none yet"}  '
                  '| 128 Hz physics | 1x playback', font=self.small, fill=(117, 224, 166))
        if self.encoder is None:
            self.encoder = subprocess.Popen([
                'ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo', '-pixel_format', 'rgb24',
                '-video_size', f'{w}x{h}', '-framerate', str(self.args.fps), '-i', '-',
                '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(self.args.out),
            ], stdin=subprocess.PIPE)
        pixels = image.tobytes()
        count = max(1, round(duration*self.args.fps))
        for _ in range(count):
            self.encoder.stdin.write(pixels)
        self.frames += count
        for name in crossings.witnessed - self.saved:
            image.save(self.args.out.with_name(self.args.out.stem+'-'+name.lower().replace(' ', '-')+'.png'))
            self.saved.add(name)

    def close(self):
        if self.encoder is not None:
            self.encoder.stdin.close()
            if self.encoder.wait(timeout=60) != 0:
                raise RuntimeError('FFmpeg failed')


def run(client, args, temporary, audit):
    state = client.call({'type': 'ready'})
    client.call({'type': 'pause', 'paused': True})
    client.call({'type': 'release_inputs'})
    if state['navigation']:
        raise RuntimeError('tour requires a fresh host without navigation bots')
    stamp = time.monotonic()
    state = client.call({'type': 'world', 'command': {'SpawnSentry': {
        'team': 'Red', 'position_m': [4.7, 3.5, 1.], 'yaw_deg': 0.,
    }}})
    owner = state['navigation'][0]['chassis']
    while not state['navigation'][0]['bake']['nodes'] or not state['team_robot']['ready']:
        if time.monotonic()-stamp > 90:
            raise RuntimeError('CAD bake/model readiness deadline exceeded')
        time.sleep(.01)
        state = client.call({'type': 'state'})
    audit.update(bake_s=time.monotonic()-stamp, bake=state['navigation'][0]['bake'], chassis=owner)
    state = client.call({'type': 'step', 'ticks': 128})
    first_tick = state['tick']
    crossings = Crossings()
    movie = None if args.check_only else Movie(client, args, temporary)
    plan_s = 0.

    def sample(label, duration=0.):
        body = next(b for b in state['chassis_all'] if b['id'] == owner)
        assert len(state['chassis_all']) == 1, 'tour must use one continuously driven chassis'
        assert body['placement_revision'] == 0, 'chassis was repositioned'
        crossings.observe(body, state['tick'])
        audit['samples'].append({'tick': state['tick'], 'leg': label,
                                 'position_m': body['pose']['translation_m'],
                                 'fold_rad': body['fold_angle_rad'],
                                 'phase': state['navigation'][0]['phase'],
                                 'message': state['navigation'][0]['message']})
        if movie:
            movie.add(state, body, label, crossings, (state['tick']-first_tick)/128,
                      plan_s, duration)

    def wait_for_path(label, started):
        nonlocal state, plan_s
        while state['navigation'][0]['phase'] in ['Planning', 'Baking']:
            if time.monotonic()-started > args.max_plan_s:
                raise RuntimeError(f'{label}: planning exceeded {args.max_plan_s}s')
            state = client.call({'type': 'state'})
        plan_s = time.monotonic()-started
        audit['plans'].append({'leg': label, 'wall_s': plan_s, **state['navigation'][0]})
        if state['navigation'][0]['phase'] != 'Following':
            raise RuntimeError(state['navigation'][0]['message'])
        print(f'{label}: path in {plan_s*1000:.0f} ms', flush=True)
        # Playback includes the observed query delay. No simulation time is
        # advanced while the host plans; capture/encoding overhead is excluded.
        if movie:
            sample(label+' — path ready', max(1/args.fps, plan_s))

    try:
        sample('Start', .75)
        for label, goal in TOUR:
            stamp = time.monotonic()
            state = client.call({'type': 'world', 'command': {'NavigateSentry': {
                'chassis': owner, 'position_m': goal,
            }}})
            wait_for_path(label, stamp)
            started_tick = state['tick']
            frame = 0
            while state['navigation'][0]['phase'] != 'Arrived':
                if state['tick']-started_tick > 180*128:
                    raise RuntimeError(f'{label}: 180 simulated second driving deadline exceeded')
                # Exact rational frame pacing; 16 fps gives eight physics ticks.
                ticks = (round((frame+1)*128/args.fps)-round(frame*128/args.fps)) if movie else 32
                stamp = time.monotonic()
                state = client.call({'type': 'step', 'ticks': ticks})
                if state['navigation'][0]['phase'] == 'Planning':
                    wait_for_path(label+' (recovery)', stamp)
                if state['navigation'][0]['phase'] not in ['Following', 'Arrived']:
                    raise RuntimeError(f'{label}: {state["navigation"][0]["message"]}')
                sample(label, ticks/128)
                frame += 1
            audit['legs'].append({'label': label, 'goal_m': goal,
                                  'simulation_s': (state['tick']-started_tick)/128})
            print(f'{label}: arrived, {audit["legs"][-1]["simulation_s"]:.1f} s driving', flush=True)
        assert set(crossings.completed) == set(PASSAGES), crossings.completed
        for frame in range(args.fps*2 if movie else 1):
            ticks = round((frame+1)*128/args.fps)-round(frame*128/args.fps) if movie else 256
            state = client.call({'type': 'step', 'ticks': ticks})
            sample('Complete — all three tunnel types crossed', ticks/128)
        body = next(b for b in state['chassis_all'] if b['id'] == owner)
        assert body['fold_angle_rad'] < .001, 'robot did not unfold after finishing'
        audit.update(completed=True, crossings=crossings.completed,
                     simulation_s=(state['tick']-first_tick)/128)
    finally:
        audit['crossings'] = crossings.completed
        if movie:
            movie.close()
            audit['video_s'] = movie.frames/args.fps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, default=Path('target/nix/debug/rm-simulator'))
    parser.add_argument('--field', type=Path, default=Path('field-2027'))
    parser.add_argument('--robot-assets', type=Path,
                        default=Path('local-assets/robot-cad-survey/bound-tunnel-sentry'))
    parser.add_argument('--out', type=Path, default=Path('local-assets/sentry-three-tunnel-tour.mp4'))
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--fps', type=int, choices=[8, 16, 32], default=16)
    parser.add_argument('--max-plan-s', type=float, default=5.)
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket() as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    audit = {'completed': False, 'tick_hz': 128, 'fps': args.fps,
             'planning_measurement': 'console goal-to-path wall latency, including transport/polling',
             'plans': [], 'legs': [], 'samples': []}
    log = args.out.with_suffix('.app.log')
    with log.open('w') as output, tempfile.TemporaryDirectory(prefix='rm-sentry-tour-') as directory:
        process = subprocess.Popen([
            str(args.app.resolve()), '--play', '--fly', '--robot', 'sentry', '--chassis', 'tunnel-omni',
            '--cad-assets', str(args.field.resolve()), '--robot-assets', str(args.robot_assets.resolve()),
            '--console', f'127.0.0.1:{port}', '--start-paused', '--window-mode', 'headless',
        ], stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+90
            while True:
                try:
                    connection = socket.create_connection(('127.0.0.1', port), 120)
                    break
                except ConnectionRefusedError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f'app startup failed; see {log}') from None
                    time.sleep(.1)
            with connection, connection.makefile('rwb') as stream:
                client = ConsoleClient(stream, emit=lambda _: None)
                try:
                    run(client, args, Path(directory), audit)
                finally:
                    client.call({'type': 'quit'})
        except BaseException as error:
            audit['error'] = str(error)
            raise
        finally:
            args.out.with_suffix('.json').write_text(json.dumps(audit, indent=2)+'\n')
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
    print(args.out.with_suffix('.json'), flush=True)


if __name__ == '__main__':
    main()
