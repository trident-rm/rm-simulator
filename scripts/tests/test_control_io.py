# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""ROS-free regressions for slow I/O, replacement, shutdown and disarming."""
import importlib.util
from pathlib import Path
import threading
import time
import unittest

_SPEC = importlib.util.spec_from_file_location(
    "control_io",
    Path(__file__).resolve().parents[2] / "integrations/ros2/scripts/control_io.py",
)
_IO = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_IO)


class ControlIoTests(unittest.TestCase):
    def test_slow_request_does_not_block_submission_and_only_latest_is_sent(self):
        entered, release, last = threading.Event(), threading.Event(), threading.Event()
        sent = []

        def send(command):
            sent.append(command)
            if command == 1:
                entered.set()
                release.wait(2)
            if command == 100:
                last.set()

        worker = _IO.CommandWorker(send)
        submitted = threading.Event()

        def callbacks():
            for value in range(2, 101):
                worker.submit(value)
            submitted.set()

        callback = threading.Thread(target=callbacks)
        try:
            worker.submit(1)
            self.assertTrue(entered.wait(1))
            callback.start()
            self.assertTrue(
                submitted.wait(0.5), "callback blocked behind in-flight HTTP"
            )
            self.assertEqual(sent, [1])
            release.set()
            self.assertTrue(last.wait(1))
            self.assertEqual(sent, [1, 100])
        finally:
            release.set()
            if callback.ident is not None:
                callback.join(2)
            worker.close()

    def test_clock_request_cannot_starve_motion(self):
        entered, release, sent = threading.Event(), threading.Event(), threading.Event()

        def clock_read():
            entered.set()
            release.wait(2)
            return {"time_ns": 10}

        clock = _IO.ClockWorker(clock_read)
        commands = _IO.CommandWorker(lambda _: sent.set())
        try:
            self.assertTrue(entered.wait(1))
            self.assertEqual(clock.take(), (None, None))
            commands.submit("move")
            self.assertTrue(sent.wait(0.5), "slow clock request blocked control")
        finally:
            release.set()
            clock.close()
            commands.close()

    def test_failed_command_is_not_retried_and_queued_motion_is_discarded(self):
        entered, release = threading.Event(), threading.Event()
        sent = []

        def send(command):
            sent.append(command)
            entered.set()
            release.wait(2)
            raise OSError("connection lost")

        worker = _IO.CommandWorker(send)
        try:
            worker.submit(1)
            self.assertTrue(entered.wait(1))
            worker.submit(2)
            release.set()
            deadline = time.monotonic() + 1
            error = None
            while error is None and time.monotonic() < deadline:
                error = worker.take_error()
                time.sleep(0.001)
            self.assertEqual(error, "connection lost")
            self.assertFalse(worker.submit(3))
            self.assertEqual(sent, [1])
        finally:
            release.set()
            worker.close()

    def test_shutdown_finishes_inflight_before_release_and_drops_pending(self):
        entered, release, closing, closed = [threading.Event() for _ in range(4)]
        order = []

        def send(command):
            entered.set()
            release.wait(2)
            order.append(command)

        worker = _IO.CommandWorker(send)

        def close():
            closing.set()
            worker.close()
            order.append("release")
            closed.set()

        closer = threading.Thread(target=close)
        try:
            worker.submit("drive")
            self.assertTrue(entered.wait(1))
            worker.submit("discarded")
            closer.start()
            self.assertTrue(closing.wait(1))
            deadline = time.monotonic() + 1
            while worker.submit("discarded") and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertFalse(worker.submit("discarded"))
            self.assertFalse(closed.is_set())
            release.set()
            self.assertTrue(closed.wait(1))
            self.assertEqual(order, ["drive", "release"])
        finally:
            release.set()
            if closer.ident is not None:
                closer.join(2)
            worker.close()


if __name__ == "__main__":
    unittest.main()
