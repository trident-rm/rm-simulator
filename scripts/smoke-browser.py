#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise a staged viewer with Chromium, HTTP faults and camera input.

Run with uv run --with playwright python scripts/smoke-browser.py --root PATH.
Install Playwright's Chromium first, or provide --executable for an existing one.
Add --server ws://127.0.0.1:5002/ to check a running 1v1 server. Allow the
HTTP origin http://127.0.0.1:8080 on that server, or choose a matching --port.
"""
import argparse
import functools
import http.server
import json
from pathlib import Path
import threading
import sys
from urllib.parse import urlencode
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


def gameplay(browser, origin, endpoint):
    """Check authoritative pilot deployment, movement and a confirmed shot."""
    context = browser.new_context(viewport={"width": 1280, "height": 800})
    blue_context = None
    try:
        page = context.new_page()
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        query = urlencode({"field": "1v1", "server": endpoint,
                           "team": "red", "name": "browser-smoke"})
        page.goto(f"{origin}/?{query}")
        page.wait_for_function("['playing','error'].includes(document.querySelector('#status').dataset.state)", timeout=120000)
        status = page.locator("#status")
        assert status.get_attribute("data-state") == "playing", (status.inner_text(), errors)
        chassis = status.get_attribute("data-chassis")
        assert chassis, "No authoritative chassis assigned"
        position = json.loads(status.get_attribute("data-position"))
        snapshot = status.get_attribute("data-snapshot")
        # Focus the canvas so browser keyboard events reach the game.
        page.locator("#sim").focus()
        page.keyboard.down("w")
        try:
            page.wait_for_function("""initial => {
                const current = JSON.parse(document.querySelector('#status').dataset.position);
                return Math.hypot(...current.slice(0, 2).map((value, i) => value - initial[i])) > 0.1;
            }""", arg=position, timeout=15000)
        finally:
            page.keyboard.up("w")
        moved = json.loads(status.get_attribute("data-position"))
        assert status.get_attribute("data-snapshot") != snapshot, "No fresh authoritative snapshot"
        page.keyboard.down("Space")
        try:
            page.wait_for_function("""() => {
                const data = document.querySelector('#status').dataset;
                return Number(data.shots) > 0 || Number(data.projectiles) > 0;
            }""", timeout=15000)
        finally:
            page.keyboard.up("Space")
        blue_context = browser.new_context(viewport={"width": 1280, "height": 800})
        blue_page = blue_context.new_page()
        blue_errors = []
        blue_page.on("pageerror", lambda error: blue_errors.append(str(error)))
        blue_query = urlencode({"field": "1v1", "server": endpoint,
                                "team": "blue", "name": "browser-smoke-blue"})
        blue_page.goto(f"{origin}/?{blue_query}")
        blue_page.wait_for_function("['playing','error'].includes(document.querySelector('#status').dataset.state)", timeout=120000)
        blue_status = blue_page.locator("#status")
        assert blue_status.get_attribute("data-state") == "playing", (blue_status.inner_text(), blue_errors)
        blue_chassis = blue_status.get_attribute("data-chassis")
        assert blue_chassis and blue_chassis != chassis, (chassis, blue_chassis)
        red_team = status.get_attribute("data-team")
        blue_team = blue_status.get_attribute("data-team")
        assert red_team and red_team.lower() == "red", red_team
        assert blue_team and blue_team.lower() == "blue", blue_team
        assert not errors, errors
        assert not blue_errors, blue_errors
        print(json.dumps({"case": "gameplay", "state": "playing", "chassis": chassis,
                          "blue_chassis": blue_chassis, "teams": [red_team, blue_team],
                          "initial_position": position, "moved_position": moved,
                          "shots": status.get_attribute("data-shots"),
                          "projectiles": status.get_attribute("data-projectiles")}), flush=True)
    finally:
        if blue_context is not None:
            blue_context.close()
        context.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("local-assets/browser"))
    parser.add_argument("--executable", help="Optional Chromium executable")
    parser.add_argument("--server", help="Optional running 1v1 practice (--no-referee) WebSocket server URL")
    parser.add_argument("--port", type=int, help="HTTP port (default: 8080 with --server, otherwise random)")
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root / "index.html").is_file():
        parser.error(f"No staged viewer at {root}")
    port = args.port if args.port is not None else (8080 if args.server else 0)
    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(Handler, directory=str(root)))
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
            if args.server:
                Handler.fault = None
                gameplay(browser, f"http://127.0.0.1:{server.server_port}", args.server)
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == "__main__":
    main()
