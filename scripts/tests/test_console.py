# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Console scenario validation, assertions and ordered wire exchanges."""
import importlib.util
import io
import json
from pathlib import Path
import socket
import threading
import unittest

spec = importlib.util.spec_from_file_location("console", Path(__file__).resolve().parents[1] / "console.py")
console = importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)


class ConsoleTests(unittest.TestCase):
    def test_repeat_expands_in_order(self):
        step = {"type": "step", "ticks": 128}
        assertion = {"assert": {"/paused": True}}
        self.assertEqual(console.expand_steps([
            {"type": "pause", "paused": True},
            {"repeat": 2, "steps": [step, assertion]},
        ]), [{"type": "pause", "paused": True}, step, assertion, step, assertion])

    def test_invalid_scripts_are_rejected_before_execution(self):
        for script in [
            {}, [None], [{}], [{"repeat": True, "steps": []}],
            [{"repeat": -1, "steps": []}], [{"repeat": 10001, "steps": []}],
            [{"repeat": 100, "steps": [{"repeat": 101, "steps": [{"type": "state"}]}]}],
            [{"assert": {}}], [{"assert": {"not/a/pointer": 1}}],
            [{"assert": {"/~2": 1}}], [{"type": "state", "value": float("nan")}],
            [{"type": "screenshot", "path": "a" * 65536}],
            [{"assert": {"/x": 1}, "type": "state"}],
        ]:
            with self.subTest(script=str(script)[:100]), self.assertRaises(ValueError):
                console.expand_steps(script)

    def test_nested_depth_limit(self):
        script = [{"type": "state"}]
        for _ in range(10):
            script = [{"repeat": 1, "steps": script}]
        with self.assertRaises(ValueError):
            console.expand_steps(script)

    def test_pointer_arrays_escaping_null_and_numbers(self):
        console.assert_state({"a/b": {"~": [None, 1.0]}}, {"/a~1b/~0/0": None, "/a~1b/~0/1": 1})
        console.assert_state({"": 2}, {"/": 2})
        console.assert_state([1, 2], {"": [1, 2]})

    def test_missing_never_equals_null_and_bool_never_equals_number(self):
        for state, expected in [
            ({}, {"/missing": None}), ({"a": None}, {"/a/x": None}),
            ([1], {"/-1": 1}), ([1], {"/00": 1}), ([1], {"/1": None}),
            ({"x": True}, {"/x": 1}), ({"x": [True]}, {"/x": [1]}),
        ]:
            with self.subTest(state=state, expected=expected), self.assertRaises(RuntimeError):
                console.assert_state(state, expected)

    def test_real_socket_queries_fresh_state_and_stops_at_failed_assertion(self):
        client, server = socket.socketpair()
        received = []
        errors = []

        def serve():
            try:
                with server, server.makefile("rwb") as stream:
                    while line := stream.readline():
                        request = json.loads(line)
                        received.append(request)
                        reply = {"id": request["id"], "ok": True, "result": {"tick": 128}}
                        stream.write((json.dumps(reply) + "\n").encode())
                        stream.flush()
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=serve)
        thread.start()
        try:
            with client, client.makefile("rwb") as stream:
                with self.assertRaisesRegex(RuntimeError, "step 3: assert /tick: expected 256, got 128"):
                    console.run_script(stream, [
                        {"type": "step", "ticks": 128},
                        {"assert": {"/tick": 128}},
                        {"assert": {"/tick": 256}},
                        {"type": "quit"},
                    ], emit=lambda _: None)
        finally:
            thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual([r["command"]["type"] for r in received], ["step", "state", "state"])
        self.assertEqual([r["id"] for r in received], [1, 2, 3])

    def test_malformed_and_rejected_replies_fail(self):
        class Stream(io.BytesIO):
            def write(self, _):
                return 0

        for response in [
            b'', b'{}', b'null\n', b'[]\n', b'{"id":true,"ok":true}\n',
            b'{"id":2,"ok":true}\n', b'{"id":1,"ok":false,"error":"denied"}\n',
        ]:
            with self.subTest(response=response), self.assertRaises(RuntimeError):
                console.run_script(Stream(response), [{"type": "state"}], emit=lambda _: None)


if __name__ == "__main__":
    unittest.main()
