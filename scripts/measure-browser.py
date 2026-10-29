#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Measure browser startup and sampled Chromium descendant RSS on macOS.

Run with uv run --with playwright python scripts/measure-browser.py --root PATH.
Each trial starts a fresh Chromium process/profile with an empty HTTP cache.
OS file caches and GPU driver caches are not flushed. RSS sums double-count shared
pages and exclude GPU allocations; Wasm capacity is recorded separately.
"""
import argparse
import hashlib
import functools
import http.server
import json
from pathlib import Path
import platform
import subprocess
import threading
import time
from playwright.sync_api import sync_playwright


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *_args):
        pass


def sample_tree(pid):
    """Return the browser's current process tree RSS, in bytes, from BSD ps."""
    rows = subprocess.check_output(['ps', '-axo', 'pid=,ppid=,rss='], text=True)
    processes = [tuple(map(int, line.split())) for line in rows.splitlines()]
    selected = {pid}
    while True:
        descendants = {child for child, parent, _ in processes if parent in selected}
        if descendants <= selected:
            break
        selected |= descendants
    return sum(rss * 1024 for child, _, rss in processes if child in selected)


INIT = r"""() => {
    window.measurement = {milestones: {}, wasmPeak: 0, wasmByScreen: {}};
    new MutationObserver(() => {
        const d = document.querySelector('#status')?.dataset;
        if (!d) return;
        const m = window.measurement;
        const mark = key => {if (!(key in m.milestones)) m.milestones[key] = performance.now();};
        if (d.screen) mark(d.screen);
        if (d.pipelinesReady === 'true') mark(`${d.screen}PipelinesReady`);
        if (JSON.parse(d.ui || '[]').some(n => n.text.includes('Field ready'))) mark('fieldPreviewReady');
        if (d.loadingError) m.error = d.loadingError;
        const capacity = Number(d.wasmMemoryBytes || 0);
        m.wasmPeak = Math.max(m.wasmPeak, capacity);
        if(d.screen) m.wasmByScreen[d.screen] = Math.max(m.wasmByScreen[d.screen] || 0, capacity);
    }).observe(document, {subtree:true, attributes:true, childList:true});
}"""


def trial(playwright, args, origin, mode, number):
    options = {'headless': True, 'args': ['--enable-unsafe-webgpu', '--use-angle=metal']}
    if args.executable:
        options['executable_path'] = args.executable
    browser = playwright.chromium.launch(**options)
    stop = threading.Event()
    samples = []
    sampler = None
    try:
        cdp = browser.new_browser_cdp_session()
        info = cdp.send('SystemInfo.getProcessInfo')['processInfo']
        pid = next(p['id'] for p in info if p['type'] == 'browser')
        baseline = sample_tree(pid)
        started = time.monotonic()
        def collect():
            while not stop.is_set():
                samples.append({'elapsed_ms': (time.monotonic()-started)*1000,
                                'rss_bytes': sample_tree(pid)})
                stop.wait(0.1)
        sampler = threading.Thread(target=collect)
        sampler.start()
        context = browser.new_context(viewport={'width':1600, 'height':1000})
        context.add_init_script(f'({INIT})()')
        page = context.new_page()
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        url = f'{origin}/?inspect=1&field=1v1' + ('&play=1' if mode == 'offline' else '')
        page.goto(url, wait_until='domcontentloaded', timeout=120000)
        target = 'InMatch' if mode == 'offline' else 'Title'
        page.wait_for_function("target => window.measurement.error || (document.querySelector('#status')?.dataset.screen === target && document.querySelector('#status')?.dataset.pipelinesReady === 'true')", arg=target, timeout=120000)
        assert not page.evaluate('window.measurement.error'), page.evaluate('window.measurement')
        if mode == 'offline':
            page.keyboard.press('F5')
        page.wait_for_timeout(args.observe_seconds * 1000)
        result = page.evaluate("""() => ({...window.measurement,
            resources: performance.getEntriesByType('resource').map(r => ({path:new URL(r.name).pathname, start_ms:r.startTime, end_ms:r.responseEnd, duration_ms:r.duration, transfer_bytes:r.transferSize, decoded_bytes:r.decodedBodySize})),
            final: {...document.querySelector('#status').dataset}})""")
        assert not errors, errors
        processes = cdp.send('SystemInfo.getProcessInfo')['processInfo']
        rss = {int(p):int(size)*1024 for p,size in (line.split() for line in subprocess.check_output(['ps','-axo','pid=,rss='],text=True).splitlines())}
        result['end_rss_by_process'] = [{'type':p['type'],'rss_bytes':rss.get(p['id'],0)} for p in processes]
        result['browser_version'] = browser.version
        result.update({'mode':mode, 'trial':number, 'baseline_rss_bytes':baseline,
                       'peak_tree_rss_bytes':max(s['rss_bytes'] for s in samples),
                       'end_tree_rss_bytes':samples[-1]['rss_bytes'], 'rss_samples':samples})
        return result
    finally:
        stop.set()
        if sampler:
            sampler.join()
        browser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--executable')
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--observe-seconds', type=float, default=10)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if platform.system() != 'Darwin':
        parser.error('RSS sampler currently requires macOS BSD ps')
    if args.trials < 1 or args.observe_seconds <= 0:
        parser.error('trials and observation duration must be positive')
    root = args.root.resolve()
    if not (root / 'index.html').is_file():
        parser.error('root must contain a staged browser application')
    server = http.server.ThreadingHTTPServer(('127.0.0.1',0), functools.partial(Handler,directory=str(root)))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    report = {'platform':platform.platform(), 'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'wasm_sha256':hashlib.sha256((root/'pkg/rm_simulator_bg.wasm').read_bytes()).hexdigest(),
              'machine':subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip(),
              'root':str(root),'viewport':[1600,1000], 'observe_seconds':args.observe_seconds,
              'notes':'Fresh Chromium process/profile per trial, empty HTTP cache; warm OS/driver caches; 100 ms sampled summed RSS includes shared pages and excludes GPU allocations. Local HTTP is uncompressed. Title and offline use the 1v1 field. inspect=1 adds observation overhead.', 'runs':[]}
    try:
        with sync_playwright() as playwright:
            for mode in ('title','offline'):
                for number in range(1,args.trials+1):
                    run = trial(playwright,args,f'http://127.0.0.1:{server.server_port}',mode,number)
                    report['runs'].append(run)
                    args.out.parent.mkdir(parents=True,exist_ok=True)
                    args.out.write_text(json.dumps(report,indent=2)+'\n')
                    print(json.dumps({k:run[k] for k in ('mode','trial','milestones','wasmPeak','peak_tree_rss_bytes','baseline_rss_bytes')}),flush=True)
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
