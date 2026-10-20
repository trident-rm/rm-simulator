#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Capture actual GPU HUD/settings layouts at several sizes through the app console.

Start a fresh paused match with --console --window-mode headless. No saved
settings are changed. Output must be a new directory; frames are normal PNGs.
"""
import argparse
import json
from pathlib import Path
import socket
import struct
from console import ConsoleClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7790)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sizes', nargs='+', default=['1280x720', '960x540', '640x360', '1920x1080'])
    args = parser.parse_args()
    sizes = []
    for size in args.sizes:
        try:
            width, height = map(int, size.lower().split('x'))
        except ValueError:
            parser.error('sizes must be WIDTHxHEIGHT')
        if not 640 <= width <= 3840 or not 360 <= height <= 2160:
            parser.error('sizes must be within 640..3840 by 360..2160')
        sizes.append((width, height))
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    records = []
    with socket.create_connection(('127.0.0.1', args.port), 120) as connection:
        with connection.makefile('rwb') as stream:
            client = ConsoleClient(stream, emit=lambda _: None)
            client.call({'type': 'ready'})
            state = client.call({'type': 'capture', 'captured': True})
            if state['ui']['blocks_input']:
                raise RuntimeError('close menus before starting the capture')
            client.call({'type': 'pause', 'paused': True})
            for width, height in sizes:
                client.call({'type': 'resize', 'width': width, 'height': height})
                client.call({'type': 'capture', 'captured': True})
                for view in ['hud', 'settings']:
                    if view == 'settings':
                        client.call({'type': 'key', 'key': 'p', 'pressed': True})
                        state = client.call({'type': 'key', 'key': 'p', 'pressed': False})
                        assert state['ui']['settings'], 'settings did not open'
                    path = folder / f'{view}-{width}x{height}.png'
                    client.call({'type': 'screenshot', 'path': str(path)})
                    header = path.read_bytes()[:24]
                    assert header[:8] == b'\x89PNG\r\n\x1a\n'
                    assert struct.unpack('>II', header[16:24]) == (width, height), 'render target size mismatch; use headless scale 1'
                    records.append({'view': view, 'size': [width, height], 'file': path.name})
                client.call({'type': 'key', 'key': 'p', 'pressed': True})
                state = client.call({'type': 'key', 'key': 'p', 'pressed': False})
                assert not state['ui']['settings'], 'settings did not close'
            client.call({'type': 'resize', 'width': 1280, 'height': 720})
    (folder / 'manifest.json').write_text(json.dumps(records, indent=2) + '\n')
    print(folder)


if __name__ == '__main__':
    main()
