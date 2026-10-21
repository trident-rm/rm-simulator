#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Record real T-input folding/unfolding at 128 Hz, checking each 500 ms stroke.

Runs the normal GPU app; source packages stay unmodified. Outputs MP4 plus an
actual-joint audit. Requires local verified robot packages, Pillow and FFmpeg.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from console import ConsoleClient

PROFILES = {
    'sentry': ('bound-tunnel-sentry', ['--robot', 'sentry', '--chassis', 'tunnel-omni']),
    'heavy': ('bound-tunnel-heavy', ['--robot', 'engineer']),
    'stairs': ('bound-stair-heavy', ['--robot', 'engineer', '--stair-heavy']),
}


def record(client, args, folder):
    from PIL import Image, ImageDraw, ImageFont
    state = client.call({'type': 'ready'})
    client.call({'type': 'pause', 'paused': True})
    client.call({'type': 'capture', 'captured': True})
    state = client.call({'type': 'step', 'ticks': 128})
    config = state['chassis']['config']
    joint = config.get('articulation') or config.get('folding') or config['climbing']
    limit = joint.get('limit_rad', joint.get('folded_angle_rad',
        joint.get('deployed_angle_rad', 0.) - joint.get('travel_angle_rad', 0.)))
    assert abs(limit / joint['speed_rad_s'] - .5) < 1e-9
    screenshot = folder / 'frame.png'
    encoder = None
    audit = []
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 24)
    client.call({'type': 'camera', 'third_person': True, 'pitch_deg': 0.})

    def frame(label, elapsed, repeat=1):
        nonlocal encoder
        client.call({'type': 'screenshot', 'path': str(screenshot)})
        with Image.open(screenshot) as source:
            image = source.convert('RGB')
            # Enlarge the real third-person view around the robot, keeping the
            # raster/geometry untouched; omit unrelated HUD panels.
            w, h = image.size
            image = image.crop((int(w*.22), int(h*.46), int(w*.72), int(h*.96))).resize(
                (w, h), Image.Resampling.LANCZOS)
        draw = ImageDraw.Draw(image)
        w, h = image.size
        draw.rectangle([0, h-70, w, h], fill=(20, 27, 36))
        draw.text((20, h-62), f'{args.profile} | {label} | {elapsed*1000:.0f} ms', font=font, fill='white')
        draw.text((20, h-32), '500 ms full stroke | actual host joints | 1x playback', font=font,
                  fill=(117, 224, 166))
        if encoder is None:
            encoder = subprocess.Popen(['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo',
                '-pixel_format', 'rgb24', '-video_size', f'{w}x{h}', '-framerate', '32',
                '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.out)], stdin=subprocess.PIPE)
        for _ in range(repeat):
            encoder.stdin.write(image.tobytes())

    try:
        frame('Deployed', 0., 24)
        client.call({'type': 'screenshot', 'path': str(args.out.with_suffix('.png'))})
        for folded, label in [(True, 'Folding'), (False, 'Unfolding')]:
            start = state['tick']
            client.call({'type': 'key', 'key': 't', 'pressed': True})
            state = client.call({'type': 'step', 'ticks': 1})
            client.call({'type': 'key', 'key': 't', 'pressed': False})
            for i in range(16):
                state = client.call({'type': 'step', 'ticks': 3 if i == 0 else 4})
                elapsed = (state['tick'] - start)/128
                angle = state['chassis']['fold_angle_rad']
                expected = limit * (elapsed/.5 if folded else 1.-elapsed/.5)
                assert abs(angle - expected) < .00001, (label, elapsed, angle, expected, state['ui'], state['chassis']['command'])
                audit.append({'phase': label, 'elapsed_s': elapsed, 'angle_rad': angle})
                frame(label, elapsed)
            frame('Stowed' if folded else 'Deployed', .5, 32)
    finally:
        if encoder:
            encoder.stdin.close()
            assert encoder.wait(timeout=60) == 0
        args.out.with_suffix('.json').write_text(json.dumps(audit, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=PROFILES, default='sentry')
    parser.add_argument('--app', type=Path, default=Path('target/nix/debug/rm-simulator'))
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    args.out = (args.out or Path(f'local-assets/folding-{args.profile}.mp4')).resolve()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket() as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    package, options = PROFILES[args.profile]
    with args.out.with_suffix('.log').open('w') as log, tempfile.TemporaryDirectory() as tmp:
        process = subprocess.Popen([str(args.app.resolve()), '--play', '--third-person', *options,
            '--cad-assets', str(Path('field-2027').resolve()), '--robot-assets',
            str(Path('local-assets/robot-cad-survey', package).resolve()), '--console',
            f'127.0.0.1:{port}', '--start-paused', '--window-mode', 'headless'],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+90
            while True:
                try:
                    connection = socket.create_connection(('127.0.0.1', port), 120)
                    break
                except ConnectionRefusedError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f'app startup failed: {args.out.with_suffix(".log")}') from None
                    time.sleep(.1)
            with connection, connection.makefile('rwb') as stream:
                client = ConsoleClient(stream, emit=lambda _: None)
                try:
                    record(client, args, Path(tmp))
                finally:
                    client.call({'type': 'quit'})
        finally:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
    print(args.out)


if __name__ == '__main__':
    main()
