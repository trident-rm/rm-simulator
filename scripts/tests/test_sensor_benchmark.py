# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Timing attribution, failure artifacts and attachment cleanup for sensor trials."""

import argparse
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    "sensor_benchmark", Path(__file__).resolve().parents[1] / "benchmark-sensors.py"
)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


def packet(sequence):
    start = sequence * 100_000_000
    return {"sequence": sequence, "start_ns": start, "end_ns": start + 100_000_000,
            "discontinuities": 1,
            "lidar": {"points": [{"sample_index": sequence * 20000, "time_ns": start}]},
            "imu": [{"time_ns": start + i * 5_000_000} for i in range(1, 21)]}


def poll(sequence):
    frame = packet(sequence)
    return {"time_ns": frame["end_ns"], "frames": [frame], "dropped_intervals": 0,
            "telemetry": {"owner_request_wait_ns": 3, "owner_poll_wall_ns": 2,
                          "losses": {"capture_intervals": 0, "ingest_errors": 0, "output_frames": 0},
                          "frames": [{"sequence": sequence, "output_queue_wait_ns": 20,
                                      "work": {"batch": sequence, "completed_frames": 1,
                                               "input_brackets": 13, "ingest_wall_ns": 10}}]}}


HEADERS = {"x-rm-sensor-host-call-ns": "8", "x-rm-sensor-encode-ns": "7"}


class SensorBenchmarkTests(unittest.TestCase):
    def test_rejects_metadata_corruption_and_misaligned_stage_counters(self):
        frame = packet(7)
        BENCH.frame_metadata(frame, 20000)
        frame["imu"][19]["time_ns"] -= 1
        with self.assertRaisesRegex(ValueError, "IMU cadence"):
            BENCH.frame_metadata(frame, 20000)
        batch = poll(7)
        BENCH.profile_metadata(batch, HEADERS)
        batch["telemetry"]["frames"][0]["sequence"] += 1
        with self.assertRaisesRegex(ValueError, "sequence mismatch"):
            BENCH.profile_metadata(batch, HEADERS)
        batch = poll(7)
        batch["telemetry"]["losses"]["output_frames"] = 1
        with self.assertRaisesRegex(ValueError, "loss total"):
            BENCH.profile_metadata(batch, HEADERS)
        with self.assertRaisesRegex(ValueError, "server durations"):
            BENCH.profile_metadata(poll(7), {**HEADERS, "x-rm-sensor-host-call-ns": "4"})

    def trial(self, root, fail=False):
        calls = []
        sequence = 0
        def post(_host, path, _body):
            nonlocal sequence
            calls.append(path)
            if path.endswith("attach-profiled"):
                return {"id": 42}, {}, {}
            if path.endswith("detach"):
                return {"ok": True}, {}, {}
            sequence += 1
            batch = poll(sequence)
            if fail:
                batch["telemetry"]["losses"]["output_frames"] = 2
                batch["dropped_intervals"] = 2
            return batch, HEADERS, {"client_headers_ns": 30, "response_bytes": 123}
        args = argparse.Namespace(output=root / "trial", host="http://localhost:1", robot=0,
                                  profile=True, carrier="Gimbal", stride=20000, streams=1,
                                  frames=2, warmup=1, timeout=1, stall_ms=0, poll_ms=0,
                                  allow_drops=False)
        with patch.object(BENCH, "post", side_effect=post):
            if fail:
                with self.assertRaisesRegex(ValueError, "sensor losses"):
                    BENCH.run(args)
            else:
                BENCH.run(args)
        self.assertEqual(calls[-1], "/api/sensors/detach")
        return json.loads((args.output / "summary.json").read_text()), args.output

    def test_success_excludes_warmup_and_keeps_raw_metadata_without_points(self):
        with tempfile.TemporaryDirectory() as temporary:
            report, output = self.trial(Path(temporary))
            self.assertEqual(report["status"], "ok")
            self.assertEqual(report["streams"][0]["received"], 3)
            self.assertEqual(report["distributions"]["ingest_wall_ns"]["n"], 2)
            trace = (output / "polls.jsonl").read_text()
            self.assertNotIn('"points"', trace)
            self.assertIn('"epoch_ns": 0', trace)

    def test_pressure_failure_still_detaches_and_writes_error_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            report, _ = self.trial(Path(temporary), fail=True)
            self.assertEqual(report["status"], "error")
            self.assertEqual(report["streams"][0]["losses"]["output_frames"], 2)
            self.assertIn("sensor losses", report["error"])

    def test_shared_work_batches_are_counted_once(self):
        original = poll
        def same_batch(sequence):
            batch = original(sequence)
            batch["telemetry"]["frames"][0]["work"]["batch"] = 1
            return batch
        with tempfile.TemporaryDirectory() as temporary, patch.dict(globals(), poll=same_batch):
            report, _ = self.trial(Path(temporary))
            self.assertEqual(report["distributions"]["ingest_wall_ns"]["n"], 1)
            self.assertEqual(report["distributions"]["output_queue_wait_ns"]["n"], 2)


if __name__ == "__main__":
    unittest.main()
