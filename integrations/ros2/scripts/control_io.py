# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Bounded HTTP workers for the ROS adapter; no ROS or simulation ownership.

Each worker holds at most one queued value plus one in flight. Network calls
run without the mailbox lock. Closing drops unsent work and waits for any in-
flight operation, so a release can safely follow the final drive request.
"""
import threading


class CommandWorker:
    """Send once, replace unsent commands, and permanently disarm on failure."""

    def __init__(self, send):
        self._send = send
        self._condition = threading.Condition()
        self._pending = None
        self._error = None
        self._failed = False
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name="nav-command-http", daemon=True
        )
        self._thread.start()

    def submit(self, command):
        """Replace one pending command without waiting for network I/O."""
        with self._condition:
            if self._closed or self._failed:
                return False
            self._pending = command
            self._condition.notify()
            return True

    def take_error(self):
        """Consume the single failure report without rearming the worker."""
        with self._condition:
            error, self._error = self._error, None
            return error

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._closed or self._pending is not None
                )
                if self._closed:
                    return
                command, self._pending = self._pending, None
            try:
                self._send(command)
            except (OSError, ValueError) as error:
                with self._condition:
                    self._failed = True
                    self._pending = None
                    self._error = str(error)
                return

    def close(self):
        """Drop queued work and finish an in-flight request before returning."""
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        self._thread.join()


class ClockWorker:
    """Poll independently of motion and retain only the latest clock reply."""

    def __init__(self, read, period_s=0.05):
        self._read = read
        self._period_s = period_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._error = None
        self._thread = threading.Thread(
            target=self._run, name="nav-clock-http", daemon=True
        )
        self._thread.start()

    def take(self):
        """Consume the newest reply/error without waiting for the HTTP request."""
        with self._lock:
            latest, error = self._latest, self._error
            self._latest = self._error = None
            return latest, error

    def _run(self):
        while not self._stop.is_set():
            try:
                status = self._read()
                with self._lock:
                    self._latest = status
                    self._error = None
            except (OSError, ValueError) as error:
                with self._lock:
                    self._error = str(error)
            self._stop.wait(self._period_s)

    def close(self):
        """Stop polling and wait for an in-flight bounded HTTP request."""
        self._stop.set()
        self._thread.join()
