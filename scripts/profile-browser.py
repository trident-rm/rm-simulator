#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Compare paused/active browser frame stalls with prediction off and on.

uv run --with playwright python scripts/profile-browser.py --root PATH --out REPORT
Reports actual frame intervals and browser long tasks, not GPU timestamp timings.
"""
import argparse
import functools
import hashlib
import http.server
import json
from pathlib import Path
import platform
import threading
from playwright.sync_api import sync_playwright


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *_args):
        pass


START = """() => {
    window.stalls = {frames: [], tasks: []};
    window.collectFrames = true;
    let last = performance.now();
    function frame(now) {
        if (!window.collectFrames) return;
        stalls.frames.push(now - last);
        last = now;
        requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
    window.taskObserver = new PerformanceObserver(list => {
        for (const task of list.getEntries()) stalls.tasks.push(task.duration);
    });
    taskObserver.observe({type: 'longtask'});
}"""
STOP = """() => {
    window.collectFrames = false;
    taskObserver.disconnect();
    return {...stalls, status: {...document.querySelector('#status').dataset}};
}"""


def run(playwright, args, origin, prediction):
    flags = ['--enable-unsafe-webgpu']
    if args.backend == 'vulkan':
        flags += ['--enable-features=Vulkan', '--use-angle=vulkan', '--disable-vulkan-surface']
    elif args.backend == 'metal':
        flags += ['--use-angle=metal']
    options = {'headless': True, 'args': flags}
    if args.executable:
        options['executable_path'] = args.executable
    browser = playwright.chromium.launch(**options)
    try:
        context = browser.new_context(viewport={'width': 1600, 'height': 1000})
        page = context.new_page()
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(f'{origin}/?inspect=1&field={args.field}&play=1&prediction={prediction}')
        page.wait_for_function("document.querySelector('#status')?.dataset.screen === 'InMatch'", timeout=180000)
        adapter = page.evaluate("""async () => {
            const adapter = await navigator.gpu.requestAdapter();
            return Object.fromEntries(['vendor', 'architecture', 'device', 'description', 'isFallbackAdapter']
                .map(key => [key, adapter.info[key]]));
        }""")
        results = []
        for phase in ('paused', 'active'):
            page.keyboard.press('F6')
            if phase == 'active':
                page.keyboard.press('F5')
                page.mouse.click(800, 500)
                page.wait_for_function("document.pointerLockElement?.id === 'sim'", timeout=15000)
                page.wait_for_timeout(100)
                page.keyboard.down('w')
                page.mouse.down()
            page.wait_for_timeout(2500)
            page.evaluate(START)
            page.wait_for_timeout(args.sample_seconds * 1000)
            result = page.evaluate(STOP)
            assert not errors, errors
            assert result['status']['paused'] == str(phase == 'paused').lower(), result['status']
            result.update(prediction=prediction, phase=phase, adapter=adapter,
                          browser=browser.version, field=args.field)
            frames = sorted(result['frames'][1:])  # The first callback starts mid-frame.
            assert frames, 'No browser frames observed'
            result['summary'] = {name: frames[int((len(frames)-1)*quantile)]
                                 for name, quantile in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]}
            result['summary']['long_tasks'] = len(result['tasks'])
            results.append(result)
            print(json.dumps({key: result[key] for key in ('prediction', 'phase', 'adapter', 'summary')}), flush=True)
            if phase == 'active':
                page.keyboard.up('w')
                page.mouse.up()
        return results
    finally:
        browser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--backend', choices=('default', 'vulkan', 'metal'), default='default')
    parser.add_argument('--executable')
    parser.add_argument('--field', choices=('rmuc', '3v3', '1v1'), default='rmuc')
    parser.add_argument('--sample-seconds', type=float, default=10)
    args = parser.parse_args()
    if args.sample_seconds <= 0:
        parser.error('sample-seconds must be positive')
    root = args.root.resolve()
    if not (root/'index.html').is_file():
        parser.error('root must contain a staged browser application')
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), functools.partial(Handler, directory=str(root)))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    report = {'platform': platform.platform(), 'backend_flags': args.backend,
              'wasm_sha256': hashlib.sha256((root/'pkg/rm_simulator_bg.wasm').read_bytes()).hexdigest(),
              'viewport': [1600, 1000], 'sample_seconds': args.sample_seconds, 'runs': []}
    try:
        with sync_playwright() as playwright:
            for prediction in (0, 1):
                report['runs'].extend(run(playwright, args, f'http://127.0.0.1:{server.server_port}', prediction))
                args.out.parent.mkdir(parents=True, exist_ok=True)
                args.out.write_text(json.dumps(report, indent=2)+'\n')
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
