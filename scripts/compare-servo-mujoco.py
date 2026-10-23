# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# /// script
# requires-python = ">=3.10"
# dependencies = ["mujoco==3.17.0"]
# ///
"""Compare servo_response CSVs with MuJoCo; no rendering or core dependency.

Run via uv run --script scripts/compare-servo-mujoco.py --help.
This is a numerical cross-check, not hardware validation or a speed benchmark.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco


def model(dt, stops):
    """Same centered 5 kg / 0.02 kg m² rotor, no contacts or passive damping."""
    limits = 'limited="true" range="-0.5 0.5"' if stops else 'limited="false"'
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco model="servo_response">
      <compiler angle="radian"/>
      <option timestep="{dt:.17g}" gravity="0 0 -9.81" integrator="Euler"
              solver="Newton" iterations="100" tolerance="1e-12"/>
      <worldbody><body name="rotor">
        <inertial pos="0 0 0" mass="5" diaginertia="0.02 0.02 0.02"/>
        <joint name="hinge" type="hinge" axis="1 0 0" {limits}
               damping="0" frictionloss="0" armature="0"/>
      </body></worldbody>
      <actuator><motor name="torque" joint="hinge" gear="1"
                ctrllimited="true" ctrlrange="-0.5 0.5"/></actuator>
    </mujoco>""")


def compare(path, hz, output):
    with path.open() as file:
        reference = list(csv.DictReader(file))
    if len(reference) != 3 * hz:
        raise ValueError(f"expected {3 * hz} rows in {path}")
    slices = math.ceil(1_000_000_000 / hz / 1_000_000)
    dt = 1 / hz / slices
    m = model(dt, True)
    d = mujoco.MjData(m)
    rows = []
    errors = {k: [] for k in ("position_rad", "velocity_rad_s", "applied_torque_nm")}
    for tick, r in enumerate(reference):
        expected_time = (tick + 1) / hz
        target = 0.3 if tick < 1.5 * hz else -0.2
        load = -0.2 if hz <= tick < 2 * hz else 0.0
        if abs(float(r["time_s"]) - expected_time) > 1e-10:
            raise ValueError("reference timestamps do not match rate")
        if float(r["target_rad"]) != target or float(r["external_torque_nm"]) != load:
            raise ValueError("reference commands do not match experiment")
        for _ in range(slices):
            # Match the Rust per-slice *explicit* bounded PD controller. Using
            # MuJoCo's implicit position shortcut would change the controller.
            d.ctrl[0] = max(-0.5, min(0.5, 4 * (target - d.qpos[0]) - 0.4 * d.qvel[0]))
            d.qfrc_applied[0] = load
            mujoco.mj_step(m, d)
        row = {
            "time_s": expected_time,
            "target_rad": target,
            "position_rad": float(d.qpos[0]),
            "velocity_rad_s": float(d.qvel[0]),
            "applied_torque_nm": float(d.actuator_force[0]),
            "external_torque_nm": load,
        }
        rows.append(row)
        for k, differences in errors.items():
            observed = float(r[k])
            if not math.isfinite(observed) or not math.isfinite(row[k]):
                raise ValueError("nonfinite reference or simulated observation")
            differences.append(row[k] - observed)
    with (output / f"mujoco-{hz}.csv").open("w") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    return {
        "hz": hz,
        "substeps": slices,
        "slice_s": dt,
        "errors": {
            k: {
                "rms": math.sqrt(sum(x * x for x in v) / len(v)),
                "max_abs": max(map(abs, v)),
            }
            for k, v in errors.items()
        },
        "final": rows[-1],
    }


def analytic(hz):
    """Independent constant-acceleration and stop checks, not trace fitting."""
    slices = math.ceil(1_000_000_000 / hz / 1_000_000)
    dt = 1 / hz / slices
    acceleration_checks = []
    for load in (0.0, -0.1, -0.2):
        m = model(dt, False)
        d = mujoco.MjData(m)
        d.ctrl[0] = 0.2
        d.qfrc_applied[0] = load
        for _ in range(64 * slices):
            mujoco.mj_step(m, d)
        expected = (0.2 + load) / 0.02 * (64 / hz)
        error = abs(float(d.qvel[0]) - expected)
        assert error < 1e-8, error
        acceleration_checks.append(
            {
                "external_torque_nm": load,
                "velocity_rad_s": float(d.qvel[0]),
                "analytic_velocity_rad_s": expected,
                "abs_error": error,
            }
        )
    m = model(dt, True)
    d = mujoco.MjData(m)
    d.ctrl[0] = 0.4
    for _ in range(2 * hz * slices):
        mujoco.mj_step(m, d)
    assert abs(float(d.qvel[0])) < 0.01
    return {
        "hz": hz,
        "acceleration": acceleration_checks,
        "stop": {
            "position_rad": float(d.qpos[0]),
            "velocity_rad_s": float(d.qvel[0]),
            "actuator_torque_nm": float(d.actuator_force[0]),
            "constraint_torque_nm": float(d.qfrc_constraint[0]),
            "penetration_rad": float(d.qpos[0]) - 0.5,
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rapier-128", type=Path, required=True)
    parser.add_argument("--rapier-1000", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    result = {
        "mujoco_version": mujoco.__version__,
        "integrator": "Euler",
        "solver": "Newton",
        "iterations": 100,
        "tolerance": 1e-12,
        "comparisons": [
            compare(args.rapier_128, 128, args.output),
            compare(args.rapier_1000, 1000, args.output),
        ],
        "analytic": [analytic(hz) for hz in (128, 1000)],
        "limitations": [
            "No speed comparison; Rust and Python loop overhead differ.",
            "Contact-free scalar rotor; does not test coupled mechanisms or tyres.",
            "MuJoCo limit uses default soft constraints; not tuned to Rapier stops.",
            "Integrator and rigid-body representation differ; agreement is not hardware truth.",
        ],
    }
    (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
