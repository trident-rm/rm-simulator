#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise the ordinary simulator UI compiled for Chromium/WebGPU.

Run with uv run --with playwright python scripts/smoke-browser.py --root PATH.
Install Playwright's Chromium first, or provide --executable for an existing one.
Pass --server ws://127.0.0.1:5002/ for a native 1v1 practice host allowing
http://127.0.0.1:8080. All controls use real keyboard/mouse events. The opt-in
inspect=1 observations come from actual application state and laid-out text.
"""
import argparse
import functools
import http.server
import json
from pathlib import Path
import sys
import threading
from urllib.parse import urlencode
from playwright.sync_api import sync_playwright


class Handler(http.server.SimpleHTTPRequestHandler):
    """Serve staged bytes, with one requested HTTP/package fault at a time."""
    fault = None

    def log_message(self, *_args):
        pass

    def do_GET(self):
        if self.fault == "missing" and self.path.endswith("/manifest.json"):
            self.send_error(404)
        elif ((self.fault == "checksum" and self.path.endswith("/floor.glb"))
              or (self.fault == "bindings" and self.path.endswith("/rmuc2027-bindings.json"))):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"changed")
        else:
            super().do_GET()


def observed(page, key):
    return page.locator("#status").get_attribute(f"data-{key}")


def texts(page):
    return json.loads(observed(page, "ui") or "[]")


def wait_text(page, value, exact=False, timeout=120000):
    """Find real Bevy text with nonzero layout in the current viewport."""
    handle = page.wait_for_function("""({value, exact}) => {
        const nodes = JSON.parse(document.querySelector('#status').dataset.ui || '[]');
        return nodes.find(node => (exact ? node.text === value : node.text.includes(value))
            && node.x > 0 && node.x < innerWidth && node.y > 0 && node.y < innerHeight);
    }""", arg={"value": value, "exact": exact}, timeout=timeout)
    return handle.json_value()


def click_text(page, value, exact=False):
    node = wait_text(page, value, exact=exact)
    page.mouse.click(node["x"], node["y"])


def wait_screen(page, screen, timeout=120000):
    page.wait_for_function("""expected => {
        const data = document.querySelector('#status').dataset;
        return data.screen === expected || Boolean(data.loadingError);
    }""", arg=screen, timeout=timeout)
    assert observed(page, "screen") == screen, (screen, observed(page, "screen"), observed(page, "loading-error"), texts(page))


def wait_frames(page, duration_ms=500):
    """Wait through browser frames without blocking the Python process."""
    page.wait_for_function("""duration => {
        const status = document.querySelector('#status');
        if (!status.dataset.smokeStarted) status.dataset.smokeStarted = performance.now();
        return performance.now() - Number(status.dataset.smokeStarted) >= duration;
    }""", arg=duration_ms)
    page.locator("#status").evaluate("element => delete element.dataset.smokeStarted")


def launch_page(browser, url):
    context = browser.new_context(viewport={"width": 1600, "height": 1000})
    page = context.new_page()
    errors = []
    def page_error(error):
        errors.append(str(error))
        print(json.dumps({"case": "page-error", "error": str(error)}), flush=True)
    page.on("pageerror", page_error)
    page.on("requestfailed", lambda request: print(json.dumps({"case": "request-failed", "url": request.url, "error": request.failure}), flush=True))
    page.goto(url)
    return context, page, errors


def native_title(browser, origin):
    context, page, errors = launch_page(browser, f"{origin}/?inspect=1")
    try:
        wait_screen(page, "Title")
        wait_text(page, "RM SIMULATOR", exact=True)
        page.wait_for_function("""() => JSON.parse(document.querySelector('#status').dataset.ui || '[]')
            .some(node => node.text.includes('Field ready') || node.text.includes('Field preview unavailable'))""", timeout=120000)
        assert not any("Field preview unavailable" in node["text"] for node in texts(page)), texts(page)
        wait_text(page, "Multiplayer", exact=True)
        assert page.locator("#pilot-hud,#crosshair,form").count() == 0, "Browser-specific HUD/form remains"
        click_text(page, "Settings", exact=True)
        wait_text(page, "Mouse sensitivity")
        page.keyboard.press("Escape")
        wait_text(page, "Multiplayer", exact=True)
        assert not errors, errors
        print(json.dumps({"case": "native-title", "settings": "passed", "html_game_widgets": 0}), flush=True)
    finally:
        context.close()


def deploy(page, side="red"):
    """Choose native staging side/robot/start position through real input."""
    wait_screen(page, "Staging")
    wait_text(page, "CHOOSE SIDE")
    page.locator("#sim").focus()
    page.keyboard.press("2" if side == "red" else "1")
    wait_text(page, "CHOOSE ROBOT")
    click_text(page, "Infantry")
    click_text(page, "Choose start position")
    wait_text(page, "START POSITION")
    page.locator("#sim").focus()
    page.keyboard.press("ArrowUp")
    wait_text(page, "START  ")
    page.keyboard.press("Enter")
    wait_screen(page, "InMatch")
    page.wait_for_function("document.querySelector('#status').dataset.pipelinesReady === 'true'", timeout=120000)
    assert observed(page, "chassis"), "No authoritative chassis after native deployment"
    wait_text(page, "PILOT")
    wait_text(page, "RED", exact=False)
    wait_text(page, "BLUE", exact=False)
    wait_text(page, "TEAM MAP")
    return observed(page, "chassis")


def pause(page):
    if page.evaluate("document.pointerLockElement !== null"):
        page.keyboard.press("Escape")
        page.wait_for_function("document.pointerLockElement === null")
    if not any(node["text"] == "Pause" for node in texts(page)):
        page.keyboard.press("Escape")
    wait_text(page, "Pause", exact=True)


def native_gameplay(browser, origin, endpoint):
    query = urlencode({"field": "1v1", "server": endpoint, "inspect": "1", "name": "native-ui-smoke"})
    context, page, errors = launch_page(browser, f"{origin}/?{query}")
    try:
        chassis = deploy(page)
        page.locator("#sim").focus()
        before_view = page.locator("#sim").screenshot()
        page.keyboard.press("v")
        wait_frames(page, 200)
        assert page.locator("#sim").screenshot() != before_view, "Native camera mode did not change the rendered view"
        page.keyboard.press("v")
        # Native hold-to-peek panels share the exact desktop controls.
        page.keyboard.down("F12")
        try:
            wait_text(page, "CONTROLS", exact=True)
        finally:
            page.keyboard.up("F12")
        page.keyboard.down("Tab")
        try:
            wait_text(page, "TEAM STATUS", exact=True)
        finally:
            page.keyboard.up("Tab")
        wait_frames(page, 200)
        position = json.loads(observed(page, "position"))
        baseline_projectile = observed(page, "last-projectile")
        page.mouse.move(800, 500)
        page.mouse.down()
        try:
            page.wait_for_function("document.pointerLockElement?.id === 'sim'", timeout=15000)
            wait_frames(page, 500)
            assert observed(page, "last-projectile") == baseline_projectile, "Capture click fired a projectile"
        finally:
            page.mouse.up()
        wait_frames(page, 100)
        page.keyboard.down("w")
        try:
            page.wait_for_function("""initial => {
                const current = JSON.parse(document.querySelector('#status').dataset.position);
                return Math.hypot(...current.slice(0, 2).map((v, i) => v - initial[i])) > 0.1;
            }""", arg=position, timeout=15000)
        finally:
            page.keyboard.up("w")
        moved = json.loads(observed(page, "position"))
        page.mouse.move(900, 500, steps=8)
        baseline_projectile = observed(page, "last-projectile")
        page.mouse.down()
        try:
            page.wait_for_function("previous => Number(document.querySelector('#status').dataset.lastProjectile) > Number(previous)",
                                   arg=baseline_projectile, timeout=15000)
        finally:
            page.mouse.up()
        pause(page)
        click_text(page, "Settings", exact=True)
        wait_text(page, "Mouse sensitivity")
        page.keyboard.press("Escape")
        wait_text(page, "Pause", exact=True)
        click_text(page, "Exit Match", exact=True)
        wait_screen(page, "Title")
        wait_text(page, "RM SIMULATOR", exact=True)
        assert observed(page, "in-match") == "false"
        # Return through native title navigation, using the remembered address.
        click_text(page, "Multiplayer", exact=True)
        click_text(page, "Join lobby / address", exact=True)
        rejoined_chassis = deploy(page)
        assert rejoined_chassis != chassis, "Host reused the departed chassis ID"
        assert page.locator("#pilot-hud,#crosshair,form").count() == 0
        assert not errors, errors
        print(json.dumps({"case": "native-gameplay", "chassis": chassis,
                          "rejoined_chassis": rejoined_chassis,
                          "horizontal_delta_m": [moved[i] - position[i] for i in range(2)],
                          "camera": "passed", "hud_and_map": "passed", "help_and_roster": "passed",
                          "capture_and_fire": "passed", "pause_settings_leave_rejoin": "passed"}), flush=True)
    except Exception:
        print(json.dumps({"case": "native-gameplay-failure", "screen": observed(page, "screen"),
                          "loading_error": observed(page, "loading-error"), "ui": texts(page),
                          "observations": page.locator("#status").evaluate("element => ({...element.dataset})"),
                          "page_errors": errors}), flush=True)
        raise
    finally:
        context.close()


def fault(browser, origin, endpoint, case):
    Handler.fault = case
    field = "rmuc" if case == "bindings" else "1v1"
    context, page, errors = launch_page(browser, f"{origin}/?{urlencode({'field': field, 'server': endpoint, 'inspect': '1'})}")
    try:
        page.wait_for_function("document.querySelector('#status').dataset.loadingError", timeout=120000)
        wait_screen(page, "Title")
        message = observed(page, "loading-error")
        expected = "HTTP 404" if case == "missing" else "package checksum mismatch"
        assert expected in message, (case, message)
        assert any(expected in node["text"] for node in texts(page)), (case, texts(page))
        assert not errors, errors
        print(json.dumps({"case": case, "screen": "Title", "error": message}), flush=True)
    finally:
        context.close()
        Handler.fault = None


def no_gpu(browser, origin):
    context = browser.new_context()
    try:
        page = context.new_page()
        page.add_init_script("Object.defineProperty(navigator,'gpu',{value:undefined})")
        page.goto(origin)
        page.wait_for_function("document.querySelector('#status').dataset.state === 'error'")
        assert "WebGPU" in page.locator("#status").inner_text()
        print(json.dumps({"case": "no-gpu", "state": "error"}), flush=True)
    finally:
        context.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("local-assets/browser-native-app"))
    parser.add_argument("--executable", help="Optional Chromium executable")
    parser.add_argument("--server", help="Running native 1v1 practice (--no-referee) WebSocket URL")
    parser.add_argument("--port", type=int, help="HTTP port (default: 8080 with --server, otherwise random)")
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root / "index.html").is_file():
        parser.error(f"No staged simulator at {root}")
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
            try:
                origin = f"http://127.0.0.1:{server.server_port}"
                native_title(browser, origin)
                if args.server:
                    native_gameplay(browser, origin, args.server)
                    for case in ("missing", "checksum", "bindings"):
                        fault(browser, origin, args.server, case)
                no_gpu(browser, origin)
            finally:
                browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == "__main__":
    main()
