#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Send ordered app console commands; waits for readiness before executing them."""
import argparse
import json
from pathlib import Path
import socket
import sys

MAX_STEPS = 10000
MAX_REPLY_BYTES = 4 * 1024 * 1024


def expand_steps(steps, depth=0):
    """Validate a script before connecting; return bounded command/assert steps."""
    if not isinstance(steps, list) or depth > 8:
        raise ValueError("script must be a list, nested at most eight levels")
    expanded = []
    for step in steps:
        if not isinstance(step, dict):
            raise ValueError("each script step must be an object")
        json.dumps(step, allow_nan=False)
        if "repeat" in step:
            count = step["repeat"]
            if set(step) != {"repeat", "steps"} or type(count) is not int or not 1 <= count <= MAX_STEPS:
                raise ValueError("repeat requires an integer 1..10000 and steps")
            body = expand_steps(step["steps"], depth + 1)
            if len(body) * count + len(expanded) > MAX_STEPS:
                raise ValueError("script exceeds 10000 expanded steps")
            expanded.extend(body * count)
        elif "assert" in step:
            if set(step) != {"assert"} or not isinstance(step["assert"], dict) or not step["assert"]:
                raise ValueError("assert requires a nonempty object of JSON pointers and expected values")
            for pointer in step["assert"]:
                pointer_tokens(pointer)
            expanded.append(step)
        elif isinstance(step.get("type"), str):
            # Reject non-finite values before sending any earlier commands.
            if len(json.dumps({"id": MAX_STEPS + 1, "command": step}, allow_nan=False).encode()) + 1 > 65536:
                raise ValueError("command exceeds console's 64 KiB request limit")
            expanded.append(step)
        else:
            raise ValueError("step requires a command type, assert, or repeat")
        if len(expanded) > MAX_STEPS:
            raise ValueError("script exceeds 10000 expanded steps")
    return expanded


def pointer_tokens(pointer):
    """Decode an RFC 6901 JSON pointer, including escaped slash and tilde."""
    if not isinstance(pointer, str) or (pointer and not pointer.startswith("/")):
        raise ValueError(f"invalid JSON pointer: {pointer!r}")
    tokens = pointer.split("/")[1:] if pointer else []
    for token in tokens:
        for i, char in enumerate(token):
            if char == "~" and (i + 1 == len(token) or token[i + 1] not in "01"):
                raise ValueError(f"invalid JSON pointer escape: {pointer!r}")
    return [token.replace("~1", "/").replace("~0", "~") for token in tokens]


def assert_state(state, expectations):
    """Compare fresh authoritative state; missing paths never equal JSON null."""
    for pointer, expected in expectations.items():
        value = state
        try:
            for token in pointer_tokens(pointer):
                if isinstance(value, list):
                    if not token.isascii() or not token.isdigit() or (len(token) > 1 and token[0] == "0"):
                        raise KeyError(token)
                    value = value[int(token)]
                elif isinstance(value, dict):
                    value = value[token]
                else:
                    raise KeyError(token)
        except (KeyError, IndexError):
            raise RuntimeError(f"assert {pointer}: path is missing; expected {expected!r}") from None
        # JSON booleans and numbers are distinct, even though Python True == 1.
        if not json_equal(value, expected):
            raise RuntimeError(f"assert {pointer}: expected {expected!r}, got {value!r}")


def json_equal(left, right):
    if isinstance(left, bool) or isinstance(right, bool):
        return type(left) is type(right) and left == right
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(json_equal(left[k], right[k]) for k in left)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(json_equal(a, b) for a, b in zip(left, right))
    return left == right


class ConsoleClient:
    """One ordered console connection, reusable by state-driven scenario runners."""
    def __init__(self, stream, emit=print):
        self.stream = stream
        self.emit = emit
        self.request_id = 0

    def call(self, command):
        """Send one command through its completion barrier and return its result."""
        self.request_id += 1
        request = {"id": self.request_id, "command": command}
        self.stream.write((json.dumps(request, allow_nan=False) + "\n").encode())
        self.stream.flush()
        line = self.stream.readline(MAX_REPLY_BYTES + 1)
        if not line or len(line) > MAX_REPLY_BYTES or not line.endswith(b"\n"):
            raise RuntimeError("console disconnected or sent an incomplete/oversized response")
        reply = json.loads(line)
        self.emit(json.dumps(reply))
        if not isinstance(reply, dict) or type(reply.get("id")) is not int or reply["id"] != self.request_id:
            raise RuntimeError("console response ID does not match the request")
        if reply.get("ok") is not True:
            raise RuntimeError(f"step {self.request_id}: {reply.get('error', 'console command failed')}")
        return reply.get("result")


def run_script(stream, steps, emit=print):
    """Execute sequentially on one connection; assertions issue fresh state queries."""
    client = ConsoleClient(stream, emit)
    for step in steps:
        command = {"type": "state"} if "assert" in step else step
        result = client.call(command)
        if "assert" in step:
            try:
                assert_state(result, step["assert"])
            except RuntimeError as error:
                raise RuntimeError(f"step {client.request_id}: {error}") from None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7790)
    parser.add_argument("--timeout", type=float, default=130)
    parser.add_argument("--no-wait", action="store_true", help="skip the initial ready command")
    parser.add_argument("--script", type=Path, help="JSON file containing an array of commands, asserts and repeats")
    parser.add_argument("commands", nargs="*", help="JSON command objects; otherwise read JSON lines from stdin")
    args = parser.parse_args()
    if args.script and args.commands:
        parser.error("--script cannot be combined with positional commands")
    if args.script:
        commands = json.loads(args.script.read_text())
    else:
        lines = args.commands if args.commands else sys.stdin
        commands = [json.loads(line) for line in lines if line.strip()]
    commands = expand_steps(commands)
    if not args.no_wait:
        commands.insert(0, {"type": "ready"})
    with socket.create_connection((args.host, args.port), args.timeout) as connection:
        with connection.makefile("rwb") as stream:
            run_script(stream, commands, emit=lambda reply: print(reply, flush=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f"console: {error}", file=sys.stderr)
        sys.exit(1)
