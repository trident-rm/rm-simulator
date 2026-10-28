#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise a staged viewer with Chromium, HTTP faults and camera input.

Run with uv run --with playwright python scripts/smoke-browser.py --root PATH.
Install Playwright's Chromium first, or provide --executable for an existing one.
"""
import argparse
import functools
import http.server
import json
from pathlib import Path
import threading
import sys
from playwright.sync_api import sync_playwright


class Handler(http.server.SimpleHTTPRequestHandler):
    """Serve real staged bytes, with one requested fault at a time."""
    fault = None

    def log_message(self, *_args):
        pass

    def do_GET(self):
        if self.fault == "missing" and self.path.endswith("/manifest.json"):
            self.send_error(404)
        elif self.fault == "checksum" and self.path.endswith("/floor.glb"):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"changed")
        else:
            super().do_GET()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("local-assets/browser"))
    parser.add_argument("--executable", help="Optional Chromium executable")
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root / "index.html").is_file():
        parser.error(f"No staged viewer at {root}")
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(Handler, directory=str(root)))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as playwright:
            options = {"headless": True, "args": ["--enable-unsafe-webgpu"]}
            if sys.platform == "darwin":
                options["args"].append("--use-angle=metal")
            if args.executable:
                options["executable_path"] = args.executable
            browser = playwright.chromium.launch(**options)
            for case in ("1v1", "3v3", "rmuc", "missing", "checksum", "no-gpu"):
                Handler.fault = case if case in ("missing", "checksum") else None
                context = browser.new_context(viewport={"width": 1280, "height": 800})
                try:
                    page = context.new_page()
                    errors = []
                    page.on("pageerror", lambda error: errors.append(str(error)))
                    if case == "no-gpu":
                        page.add_init_script("Object.defineProperty(navigator,'gpu',{value:undefined})")
                    field = case if case in ("1v1", "3v3", "rmuc") else "1v1"
                    page.goto(f"http://127.0.0.1:{server.server_port}/?field={field}")
                    page.wait_for_function("['ready','error'].includes(document.querySelector('#status').dataset.state)", timeout=120000)
                    state = page.locator("#status").get_attribute("data-state")
                    text = page.locator("#status").inner_text()
                    if case in ("1v1", "3v3", "rmuc"):
                        assert state == "ready", (case, text, errors)
                        before = page.screenshot()
                        page.mouse.move(650, 400)
                        page.mouse.down()
                        page.mouse.move(850, 500, steps=12)
                        page.mouse.up()
                        assert page.screenshot() != before, f"Camera did not move: {case}"
                    else:
                        assert state == "error", (case, text)
                        expected = {"missing": "HTTP 404", "checksum": "checksum mismatch", "no-gpu": "WebGPU"}[case]
                        assert expected in text, (case, text)
                    assert not errors, (case, errors)
                    print(json.dumps({"case": case, "state": state, "text": text}), flush=True)
                finally:
                    context.close()
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == "__main__":
    main()
