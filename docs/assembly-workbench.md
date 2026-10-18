<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Cartesian assembly workbench

The standalone `engineer_assembly` example is the first interactive assembly
prototype. A Cartesian tool moves at 0.12 m/s per axis and rotates at 0.6 rad/s
per axis on the ordinary 128 Hz clock. An elbow-up serial arm follows it using
exact inverse kinematics. Unreachable targets are refused rather than projected.
The chassis stays stationary; the carried energy unit starts in the gripper.

```sh
nix develop -c cargo run -p rm-simulator-app --example engineer_assembly -- \
  --robot-assets local-assets/robot-cad-survey/bound-tunnel-heavy
```

The optional source package must pass the app's existing checksum/profile
verification and be PIE. Its chassis and wheels keep their source materials;
the original arm, camera and UWB groups are hidden. The workbench arm, socket
and armor blocks are explicit schematic proxies, not a fitted PIE manipulation
binding or an export of the Tech Core. Without the package, a box represents
the chassis. Source ownership and terms remain in the package CAD-NOTICE.

| Key | Action |
|---|---|
| W / S | Tool −x / +x, toward / away from this fixture |
| A / D | Tool +y / −y |
| R / F | Tool +z / −z |
| Up / Down | Increase / decrease P rotation |
| Left / Right | Increase / decrease Q rotation |
| Z / X | Off-axis roll, useful for testing misalignment |
| E | Confirm assembly |
| G | Release / grip the unit; releasing a seated unit fails |
| Space | Pause the workbench clock |
| 1 / 2 / 3 | Restart at that difficulty |
| Escape | Exit |

Move W until seated. At difficulty two or three, lift 100 mm using R. For
difficulty three, rotate P upward 90 degrees, then rotate Q to the displayed
30-degree target and hold alignment for 400 ms. Press E to confirm. The cyan
guide shows the insertion and lifting directions. Moving sideways after seating
removes the unit; this prototype does not supply guide contact forces.

`--demo` drives the same rate-limited tool through the sequence. `--difficulty`
chooses 1–3. `--headless` runs that demonstration without a GPU, prints the
serialized final authority and exits unsuccessfully if it cannot finish:

```sh
nix develop -c cargo run -p rm-simulator-app --example engineer_assembly -- \
  --headless --difficulty 3
```

`--screenshot local-assets/assembly.png` implies the demonstration and captures
the completed scene. These are example options, not `rm-simulator` CLI options.

## Mechanism authority

`gameplay::rmuc_2027::manipulation::Attempt` consumes simultaneous, authoritative
mechanism observations and explicit monotonic nanosecond timestamps. Callers
must measure insertion, lift, P/Q bearing angles, axis/guide error and seating;
these are not world Euler angles or pilot-provided certificates. The workbench
derives its observations from the actual commanded tool pose. A later physics
adapter must derive them from the source socket/bearing frames and contacts.

UC §5.4.3 / Figures 5-12–5-15 supply the ordered 100 mm insertion, 100 mm lift,
90-degree P raise, five-degree Q tolerance, confirmation and one-second rejected
confirmation lockout. The simulator selects 400 ms from the manual's 0.3–0.5 s
indication range. Three-millimetre travel/guide tolerance and five-degree initial
axis/P tolerance are workbench policies; the manual does not specify them.

Difficulty four's evaluator tracks both cores and five-second paired insertion,
lift, alignment and confirmation milestones; P raising is exempt. Five seconds
is inclusive, while the 45-second overall completion deadline is exclusive.
Continuous RFID absence fails at fifteen seconds and death/removal fails
immediately. Timer expiry precedes an observation at the same time. Lower
difficulties have no evaluator timeout, unlike the separately documented live
button prototype. The interactive workbench currently provides only one arm and
one core and consequently exposes difficulties 1–3.

The evaluator serializes its hidden dwell, timing, lockout and milestone state.
Tests exercise correct and incorrect sequences, paired boundaries, contact
renewal, withdrawal, death and restore. `physics::manipulator::Arm` implements
the independent prescribed position solver; it does not enforce gameplay.

## Remaining integration

This is an isolated workbench, without match networking, Heavy deployment,
progression or rewards. It cannot certify a live match. The existing referee
Start/Finish/Cancel path remains the only live assembly progression authority.

Before using this with a real Heavy: bind the CAD Tech Core socket and P/Q
subtrees and their pivots; calibrate a donor's full manipulation chain, wrist,
limits and reach; replace free-space sensor proxies with geometric attachment,
guide/contact and withdrawal checks; put commands and full restore state through
the existing host protocol; and connect Heavy ownership, RFID, defeat/recovery,
invincibility and rewards. Difficulty four additionally needs two energy units
and a supported manipulation strategy. Target poses currently use a fixed,
level socket rather than sampling Table 5-10's full spherical pose range.
