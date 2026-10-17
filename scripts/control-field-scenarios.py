#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise real 2027 RFID with two training bots; resets the local referee match."""
import argparse
import json
import math
from pathlib import Path
import socket
import sys

from console import ConsoleClient, assert_state


def inside(p, polygon):
    """Match the world's even/odd polygon test, independent of presentation."""
    hit = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]:
            hit = not hit
    return hit


def positions(zone):
    """Find two interior positions separated enough for ordinary chassis bodies."""
    polygon = zone["polygon_m"]
    lo = [min(p[i] for p in polygon) for i in range(2)]
    hi = [max(p[i] for p in polygon) for i in range(2)]
    samples = []
    for x in range(1, 20):
        for y in range(1, 20):
            p = [lo[0] + (hi[0] - lo[0]) * x / 20, lo[1] + (hi[1] - lo[1]) * y / 20]
            # Keep 12 cm away from card-footprint edges when possible.
            if all(inside([p[0] + dx, p[1] + dy], polygon) for dx, dy in [(0,0), (.12,0), (-.12,0), (0,.12), (0,-.12)]):
                samples.append(p)
    if len(samples) < 2:
        raise RuntimeError("zone has no two usable interior samples")
    if zone.get("point") == "Fortress":
        # The footprint includes sloping ramps. Prefer central top-deck samples
        # rather than opposite outer corners where a parked chassis slides off.
        def clearance(p):
            distances = []
            for a,b in zip(polygon, polygon[1:] + polygon[:1]):
                dx,dy = b[0]-a[0], b[1]-a[1]
                if dx == 0 and dy == 0:
                    continue
                t = max(0, min(1, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/(dx*dx + dy*dy)))
                distances.append(math.dist(p, [a[0]+t*dx, a[1]+t*dy]))
            return min(distances)
        ranked = [(p, clearance(p)) for p in samples]
        # The Fortress plateau's long axis is field x. A shared central y row
        # keeps wheels on the flat top rather than across the shorter ramp axis.
        pairs = [(a,b,min(ca,cb),ca+cb) for a,ca in ranked for b,cb in ranked
                 if math.dist(a,b) >= .8 and abs(a[1]-b[1]) < 1e-9]
        if not pairs:
            raise RuntimeError("Fortress top has no separated interior pair")
        center_y = (lo[1]+hi[1])/2
        a,b,_,_ = max(pairs, key=lambda pair:(-abs(pair[0][1]-center_y),pair[2],pair[3],-math.dist(pair[0],pair[1])))
    else:
        a, b = max(((a,b) for a in samples for b in samples), key=lambda pair: math.dist(*pair))
    if math.dist(a,b) < .8:
        raise RuntimeError("zone is too small for two separated chassis")
    return [[*p, zone["floor_m"] + .4] for p in [a,b]]


def run(client, screenshots=None):
    state = client.call({"type": "ready"})
    if state.get("role") != "Referee" or not state.get("rmuc_2027_config"):
        raise RuntimeError("requires a local --referee app and a bound 2027 field")
    client.call({"type": "pause", "paused": True})
    def world(command):
        return client.call({"type": "world", "command": command})
    world({"Referee": "ResetMatch"})
    world("ClearBots")
    ids = {}
    for team in ["Red", "Blue"]:
        before = {r["id"] for r in client.call({"type":"state"})["rmuc_2027"]["robots"]}
        state = world({"SpawnBot": {"team": team, "spin_rad_s": 0}})
        created = [r["id"] for r in state["rmuc_2027"]["robots"] if r["id"] not in before]
        if len(created) != 1:
            raise RuntimeError("spawn did not create exactly one native robot")
        ids[team] = created[0]
    zones = state["rmuc_2027_config"]["zones"]
    def zone(side, point):
        return next(z for z in zones if z["side"] == side and z["point"] == point)
    def place(team, position):
        return world({"PlaceBot": {"chassis": ids[team], "position_m": position, "yaw_deg": 0}})
    def step(ticks):
        return client.call({"type": "step", "ticks": ticks})
    def check(expectations):
        state = client.call({"type": "state"})
        assert_state(state, expectations)
        return state
    def capture(name):
        if screenshots:
            eye = [focus[0] + 5, focus[1] - 6, focus[2] + 5]
            client.call({"type":"camera", "position_m":eye, "yaw_deg":math.degrees(math.atan2(6,-5)),
                         "pitch_deg":-math.degrees(math.atan2(5, math.hypot(5,6)))})
            client.call({"type":"inspect", "zones":True, "zone_labels":True})
            client.call({"type":"screenshot", "path":str(screenshots / f"{name}.png")})
    def start():
        world({"Referee": "ResetMatch"})
        world({"Referee": "StartMatch"})
        step(640)
    def park(team):
        # Their own base is far from the opposing point being tested.
        place(team, positions(zone(team, "Base"))[0])

    # Both sides: actual chassis contact contests Base, departure has 2 s grace,
    # then 5 s uncontested capture. No RFID/control state is injected.
    for side in ["Red", "Blue"]:
        territory = 2 + (side == "Blue")
        enemy = "Blue" if side == "Red" else "Red"
        a,b = positions(zone(side, "Base"))
        focus = [(x+y)/2 for x,y in zip(a,b)]
        park("Red"); park("Blue"); start()
        place(enemy,a); place(side,b)
        step(128)
        check({f"/rmuc_2027/control/state/territories/{territory}/activations/0": [True,True],
               f"/rmuc_2027/control/state/territories/{territory}/progress_ns": 0})
        capture(f"{side.lower()}-base-contested")
        # Use the other side's Base to leave this footprint.
        place(side, positions(zone(enemy, "Base"))[1])
        step(128)
        check({f"/rmuc_2027/control/state/territories/{territory}/progress_ns":0})
        step(6 * 128)
        check({f"/rmuc_2027/control/state/territories/{territory}/control": {"Temporary":enemy}})
        capture(f"{side.lower()}-base-captured")
        park(enemy)
        step(7 * 128)
        check({f"/rmuc_2027/control/state/territories/{territory}/control": {"Full":side}})

    # Highland arbitration and same-round combat: defender arrives first,
    # then attacker takes the latch after defender leaves and grace expires.
    park("Red"); park("Blue"); start()
    a,b = positions(zone("Blue", "CentralHighland"))
    focus = [(x+y)/2 for x,y in zip(a,b)]
    place("Blue",a); step(16)
    place("Red",b); step(16)
    state = check({"/rmuc_2027/highland_team/1":"Blue"})
    index = next(i for i,r in enumerate(state["control_diagnostics"]) if r["id"] == ids["Red"])
    check({f"/control_diagnostics/{index}/points/5/status":"HighlandHeld"})
    capture("highland-held")
    park("Blue"); step(7 * 128)
    check({"/rmuc_2027/highland_team/1":"Red",
           "/rmuc_2027/control/state/territories/1/control":{"Temporary":"Red"}})
    # Keep the attacker in place and destroy the outpost to secure full control.
    world({"Referee":{"SetOutpostHp":{"outpost":1,"hp":0}}})
    check({"/rmuc_2027/control/state/territories/1/control":{"Full":"Red"},
           "/rmuc_2027/outpost_hp/1":500})
    capture("highland-transferred")

    # Defending Fortress benefits use historical Base loss after a repair.
    park("Red"); park("Blue"); start()
    world({"Referee":{"SetOutpostHp":{"outpost":1,"hp":0}}})
    step(5 * 128)
    for hp in [4000, 10000]:
        world({"Referee":{"SetBaseHp":{"team":"Blue", "hp":hp, "shield_hp":0}}})
    a,b = positions(zone("Blue", "Fortress"))
    focus = [(x+y)/2 for x,y in zip(a,b)]
    place("Blue",focus); step(16)
    state = client.call({"type":"state"})
    blue = next(i for i,r in enumerate(state["control_diagnostics"]) if r["id"] == ids["Blue"])
    red = next(i for i,r in enumerate(state["control_diagnostics"]) if r["id"] == ids["Red"])
    check({"/rmuc_2027/base_lowest_hp/1":4000,
           f"/control_diagnostics/{blue}/points/7/status":"Active",
           f"/control_diagnostics/{blue}/effects/defense_pct":50,
           f"/control_diagnostics/{blue}/effects/cooling_per_s":87,
           f"/control_diagnostics/{blue}/effects/fortress_reserve":500})
    capture("fortress-defending")
    # Opposing robots activate the same Fortress after 3:00. Vulnerability
    # disappears when its protective armor opens after ten seconds.
    step(180 * 128)
    place("Blue",a)
    place("Red",b); step(16)
    check({f"/control_diagnostics/{red}/points/7/status":"Active",
           f"/control_diagnostics/{blue}/points/7/status":"Active",
           f"/control_diagnostics/{red}/effects/vulnerability_pct":100,
           "/rmuc_2027/base_open/1":False})
    capture("fortress-attacking")
    # Both teams can activate concurrently. For the sustained ten-second hold,
    # stage the defender out and center the attacker on the small flat plateau;
    # two chassis straddling its ramps need active driving to stay stationary.
    park("Blue")
    place("Red",focus)
    step(1)
    step(9 * 128)
    check({"/rmuc_2027/base_open/1":False})
    step(128)
    check({"/rmuc_2027/base_open/1":True,
           f"/control_diagnostics/{red}/points/7/status":"Active",
           f"/control_diagnostics/{red}/effects/vulnerability_pct":0})
    capture("fortress-open")
    # Leave bots and match clean; never remove a pilot chassis.
    world("ClearBots")
    world({"Referee":"ResetMatch"})
    print(json.dumps({"scenario":"control-field-2027", "ok":True, "robots":ids}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7790)
    parser.add_argument("--screenshots", type=Path)
    args = parser.parse_args()
    if args.screenshots:
        args.screenshots = args.screenshots.resolve()
        args.screenshots.mkdir(parents=True, exist_ok=True)
    with socket.create_connection((args.host, args.port), 130) as connection:
        with connection.makefile("rwb") as stream:
            run(ConsoleClient(stream, emit=lambda reply: print(reply, flush=True)), args.screenshots)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, StopIteration) as error:
        print(f"control-field-scenarios: {error}", file=sys.stderr)
        sys.exit(1)
