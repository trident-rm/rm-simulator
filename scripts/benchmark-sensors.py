#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Measure an existing host's sensor path without controlling its world or clock.

Standard library only. Attachments are temporary; raw point arrays are never
written. Host durations and simulation stamps remain separate throughout.
"""

import argparse
import hashlib
import http.client
import json
import math
import os
import platform
import subprocess
import time
import urllib.parse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIMIT = 32 * 1024 * 1024


def distribution(values):
    """Nearest-rank quantiles, in the same units as the input, without filtering."""
    if not values:
        return {"n": 0}
    ordered = sorted(values)
    if not all(math.isfinite(v) and v >= 0 for v in ordered):
        raise ValueError("invalid duration")
    return {"n": len(ordered), "min": ordered[0], **{
        label: ordered[math.ceil(q * len(ordered)) - 1]
        for label, q in [("p50", .5), ("p95", .95), ("p99", .99)]
    }, "max": ordered[-1]}


def post(host, path, body, timeout=3):
    """One connection per request, matching the server's Connection: close policy."""
    address = urllib.parse.urlsplit(host)
    if address.scheme != "http" or not address.hostname or address.path not in ("", "/"):
        raise ValueError("host must be an http://HOST:PORT origin")
    connection = http.client.HTTPConnection(address.hostname, address.port, timeout=timeout)
    start = time.monotonic_ns()
    try:
        connection.request("POST", path, json.dumps(body).encode(),
                           {"Content-Type": "application/json"})
        response = connection.getresponse()
        headers_at = time.monotonic_ns()
        headers = {k.lower(): v for k, v in response.getheaders()}
        data = bytearray()
        while chunk := response.read1(min(65536, LIMIT + 1 - len(data))):
            data.extend(chunk)
            if len(data) > LIMIT:
                raise ValueError("sensor response exceeds 32 MiB")
            if (time.monotonic_ns() - start) / 1e9 > timeout:
                raise TimeoutError("sensor response exceeded request deadline")
        if response.length not in (None, 0):
            raise ValueError("truncated HTTP response body")
        read_at = time.monotonic_ns()
        if response.status != 200:
            raise ValueError(f"{path}: HTTP {response.status}: {bytes(data[:512])!r}")
        value = json.loads(data)
        decoded_at = time.monotonic_ns()
        return value, headers, {
            "client_headers_ns": headers_at - start,
            "client_body_read_ns": read_at - headers_at,
            "client_json_decode_ns": decoded_at - read_at,
            "response_bytes": len(data),
        }
    finally:
        connection.close()


def frame_metadata(frame, stride):
    """Validate default periods, retained emission indices and all IMU stamps.

    Does not validate every point value: acquisition/measurement tests cover that.
    Returns simulation time metadata, never a wall-time estimate.
    """
    start, end, sequence = frame["start_ns"], frame["end_ns"], frame["sequence"]
    epoch = start - sequence * 100_000_000
    if end - start != 100_000_000 or epoch < 0:
        raise ValueError("invalid packet simulation stamps")
    first = ((start - epoch) // 5000 + stride - 1) // stride * stride
    stop = (end - epoch) // 5000
    points = frame["lidar"]["points"]
    if len(points) != (stop - first + stride - 1) // stride:
        raise ValueError("incomplete LiDAR packet")
    for index in {0, len(points) // 2, len(points) - 1}:
        point = points[index]
        emission = first + index * stride
        if point["sample_index"] != emission or point["time_ns"] != epoch + emission * 5000:
            raise ValueError("invalid retained emission index/time")
    if [s["time_ns"] for s in frame["imu"]] != [start + i * 5_000_000 for i in range(1, 21)]:
        raise ValueError("invalid IMU cadence")
    return {"sequence": sequence, "start_ns": start, "end_ns": end,
            "epoch_ns": epoch, "discontinuities": frame["discontinuities"],
            "lidar_emissions": len(points), "imu_samples": len(frame["imu"])}


def profile_metadata(batch, headers):
    """Require aligned telemetry and matching stage totals; do not infer causation."""
    telemetry = batch["telemetry"]
    if len(telemetry["frames"]) != len(batch["frames"]):
        raise ValueError("telemetry/frame count mismatch")
    for frame, timing in zip(batch["frames"], telemetry["frames"]):
        if frame["sequence"] != timing["sequence"]:
            raise ValueError("telemetry/frame sequence mismatch")
    if sum(telemetry["losses"].values()) != batch["dropped_intervals"]:
        raise ValueError("stage loss total mismatch")
    host = int(headers["x-rm-sensor-host-call-ns"])
    encode = int(headers["x-rm-sensor-encode-ns"])
    wait, poll = telemetry["owner_request_wait_ns"], telemetry["owner_poll_wall_ns"]
    if not isinstance(wait, int) or min(host, encode, wait, poll) < 0 or host < wait + poll:
        raise ValueError("inconsistent server durations")
    return {"host_call_ns": host, "json_encode_ns": encode,
            "owner_request_wait_ns": wait, "owner_poll_ns": poll}


def source_identity():
    """Commit plus hashes of relevant possibly uncommitted source, not binary proof."""
    files = [Path(__file__).resolve(), *(
        ROOT / "crates/rm-sim-server/src" / name
        for name in ("sensor_stream.rs", "host.rs", "http.rs", "simulation.rs")
    )]
    return {
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    }


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "options": {**vars(args), "output": str(args.output)},
              "platform": platform.platform(), "cpu": platform.processor(),
              "python": platform.python_version(),
              "logical_cpus": os.cpu_count(), "load_start": os.getloadavg(),
              "source": source_identity(), "streams": []}
    attached, samples, work_seen = [], {}, set()
    wall_start, cpu_start = time.monotonic_ns(), time.process_time_ns()
    error = None
    suffix = "-profiled" if args.profile else ""
    def sample(name, value):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"invalid measurement: {name}")
        samples.setdefault(name, []).append(value)
    try:
        for _ in range(args.streams):
            config = {"robot": args.robot, "stride": args.stride, "carrier": args.carrier}
            attachment = post(args.host, "/api/sensors/attach" + suffix, config)[0]["id"]
            attached.append(attachment)
            report["streams"].append({"attachment": attachment, "received": 0, "measured": 0,
                                      "sequence_gaps": 0, "dropped_intervals": 0,
                                      "last_sequence": None, "epoch_ns": None})
        deadline = time.monotonic() + args.timeout
        stalled = False
        with (args.output / "polls.jsonl").open("w") as trace:
            while any(s["measured"] < args.frames for s in report["streams"]):
                if time.monotonic() >= deadline:
                    raise TimeoutError("did not receive requested frames before run deadline")
                if (args.stall_ms and not stalled
                        and all(s["measured"] >= max(1, args.frames // 2) for s in report["streams"])):
                    time.sleep(args.stall_ms / 1000)
                    stalled = True
                for stream in report["streams"]:
                    # Continue polling completed streams to retain their attachments.
                    batch, headers, timing = post(args.host, "/api/sensors/poll" + suffix, stream["attachment"])
                    server = profile_metadata(batch, headers) if args.profile else {}
                    measured_poll = stream["received"] >= args.warmup
                    metadata = []
                    for index, frame in enumerate(batch["frames"]):
                        meta = frame_metadata(frame, args.stride)
                        if meta["end_ns"] > batch["time_ns"]:
                            raise ValueError("packet is newer than the simulation clock")
                        sequence, previous = meta["sequence"], stream["last_sequence"]
                        if previous is not None:
                            if sequence <= previous:
                                raise ValueError("nonmonotonic packet sequence")
                            stream["sequence_gaps"] += sequence - previous - 1
                        if stream["epoch_ns"] not in (None, meta["epoch_ns"]):
                            raise ValueError("attachment epoch changed")
                        stream["epoch_ns"], stream["last_sequence"] = meta["epoch_ns"], sequence
                        stream["received"] += 1
                        meta["measured"] = stream["received"] > args.warmup
                        if meta["measured"]:
                            stream["measured"] += 1
                            sample("simulation_packet_age_ns", batch["time_ns"] - meta["end_ns"])
                            if args.profile:
                                ft = batch["telemetry"]["frames"][index]
                                sample("output_queue_wait_ns", ft["output_queue_wait_ns"])
                                key = (stream["attachment"], ft["work"]["batch"])
                                if key not in work_seen:
                                    work_seen.add(key)
                                    for name, value in ft["work"].items():
                                        if name.endswith("_ns"):
                                            sample(name, value)
                        metadata.append(meta)
                    stream["dropped_intervals"] = batch["dropped_intervals"]
                    if args.profile:
                        stream["losses"] = batch["telemetry"]["losses"]
                    # A poll crossing warmup is included; both its frames share encoding.
                    if measured_poll or any(m["measured"] for m in metadata):
                        for name, value in {**timing, **server}.items():
                            sample(name, value)
                            sample(("nonempty_" if metadata else "empty_") + name, value)
                    trace.write(json.dumps({"attachment": stream["attachment"],
                                            "time_ns": batch["time_ns"], "frames": metadata,
                                            "timing": timing, "server": server,
                                            "dropped_intervals": batch["dropped_intervals"],
                                            "telemetry": batch.get("telemetry")}) + "\n")
                time.sleep(args.poll_ms / 1000)
        if not args.allow_drops and any(s["dropped_intervals"] or s["sequence_gaps"] for s in report["streams"]):
            raise ValueError("sensor losses observed; inspect stage counters (or use --allow-drops for pressure trials)")
        report["status"] = "ok"
    except Exception as exc:
        error = exc
        report.update(status="error", error=f"{type(exc).__name__}: {exc}")
    finally:
        cleanup_errors = []
        for attachment in attached:
            try:
                post(args.host, "/api/sensors/detach", attachment)
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
            report["status"] = "error"
            error = error or RuntimeError("attachment cleanup failed")
        report.update(wall_ns=time.monotonic_ns() - wall_start,
                      client_process_cpu_ns=time.process_time_ns() - cpu_start,
                      load_end=os.getloadavg(),
                      distributions={name: distribution(values) for name, values in samples.items()})
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    if error:
        raise error
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="Existing operator HTTP origin, http://HOST:PORT")
    parser.add_argument("--robot", type=int, required=True, help="Existing live chassis ID")
    parser.add_argument("--profile", action="store_true", help="Opt in to profiled attachment/poll endpoints")
    parser.add_argument("--carrier", choices=["Chassis", "Gimbal"], default="Chassis")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--streams", type=int, choices=range(1, 5), default=1)
    parser.add_argument("--frames", type=int, default=60, help="Minimum measured frames per stream")
    parser.add_argument("--warmup", type=int, default=5, help="Discard first N received frames per stream")
    parser.add_argument("--poll-ms", type=float, default=10)
    parser.add_argument("--stall-ms", type=float, default=0, help="One intentional mid-run consumer stall, at most 1000 ms")
    parser.add_argument("--allow-drops", action="store_true", help="Pressure trial: retain losses without failing")
    parser.add_argument("--timeout", type=float, default=60, help="Run deadline in wall seconds, plus bounded request/cleanup time")
    parser.add_argument("--output", type=Path, required=True, help="New local artifact directory")
    args = parser.parse_args()
    if (args.robot < 0 or not 1 <= args.stride <= 20000 or args.frames < 1 or args.warmup < 0
            or not math.isfinite(args.poll_ms) or not 0 <= args.poll_ms <= 1000
            or not math.isfinite(args.stall_ms) or not 0 <= args.stall_ms <= 1000
            or not math.isfinite(args.timeout) or not 1 <= args.timeout <= 3600):
        parser.error("invalid robot, stride, frame count, polling, stall or timeout value")
    report = run(args)
    print(json.dumps({"status": report["status"], "streams": report["streams"],
                      "summary": str(args.output / "summary.json")}, indent=2))


if __name__ == "__main__":
    main()
