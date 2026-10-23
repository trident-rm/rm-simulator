<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Tunnel RFID calibration

Native nine-card tunnel sequences use UC §5.6.4.5/Table 5-27, independently of
navigation staging points. The original Figure 5-23 registration put Highland
pads on diagonal lines that did not match the installed field's straight lanes.
`server::zone_catalog` now aligns them to the source-derived Low package, source
SHA-256 `a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0`.

These measurements describe thin CAD marking rectangles, not verified antenna
radiation patterns. The native binding remains an explicit approximation.
Endpoint naming A/B has no direction restriction. Blue is the point mirror of
Red; all coordinates below are world FLU metres.

| Tunnel | End A center x/y | Interior x/y | End B center x/y | Initial floor reference |
|---|---|---|---|---|
| Road | 4.6871, 4.87535 | 4.6868, 5.3452 | 4.68645, 5.81504 | −0.10 / 0.01 / 0.047 |
| Near Highland | −1.35594, 5.95629 | −0.92627, 5.95629 | −0.49659, 5.95629 | −0.1563 |
| Far Highland | −0.40594, 6.92491 | 0.02373, 6.92491 | 0.45341, 6.92491 | 0.0176 |

Road End A is a sloped rectangle in `central-tunnel-side-red`; End B is source
body 3783190. Near endpoints are bodies 3785286 and 3784238. Far endpoints
are disconnected 250 mm white rectangles retained in body 3776011. Duplicate
surface candidates may appear in the floor coverage guard and arena mesh;
they do not represent additional cards. Interior pads are inferred under the
surveyed roofs, near x=−0.93 for Near and x=0.02 for Far, rather than under
an arbitrary midpoint of the navigation approach.

Reproduce rectangle measurements without CAD libraries:

```sh
python3 scripts/survey-tunnel-rfid.py field-2027
nix develop -c cargo run -p rm-sim-server --example survey_tunnel_cards --locked
```

Detection approximates the robot's antenna by its body center. Endpoint pads
are 250 mm along travel; inferred interior pads are 160 mm. All use a 600 mm
transverse band, allowing the demonstrated off-center lane routes. These widths
are simulator policy, not rulebook dimensions. A 260 mm body-center height band
and the existing source-floor fitting distinguish under-roof travel from travel
above the deck. Source fitting never uses a roof as the floor for an interior
card. Navigation retains its independent, longer staging/clearance seeds.

The geometric rule regression samples continuous lines in both directions on
all six passages. It checks valid 2.5 s crossings, rejected 8 s crossings and
rejected routes above the roof. The actual app/host/Rapier navigation regression
adds measured RFID observations and awarded benefits:

```sh
python3 scripts/render-sentry-navigation-video.py --check-only --rules \
  --out local-assets/sentry-rfid-rules.mp4
```

The tour physically crosses all three types without repositioning and receives
ThroughTunnel on both Highland lanes (including the Far return). With faster
navigation, the Road crossing can also meet the three-second card window. The
native engine awards it only when the observed sequence meets that deadline;
physical passage success alone is insufficient. The audit records contacts,
sequence deadlines and actual benefit awards; rewards always come from native
per-tick contacts.
