#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Verify ROS -> HTTP -> live CAD host motion, command loss and manual takeover."""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

from control import request


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def stop(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[3]
    parser.add_argument(
        "--server-bin", type=Path, default=root / "target/nix/debug/rm-simulator-server"
    )
    args = parser.parse_args()
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_DOMAIN_ID"] = "74"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    host = f"http://127.0.0.1:{free_port()}"
    with tempfile.TemporaryFile(mode="w+") as server_log, tempfile.TemporaryFile(
        mode="w+"
    ) as bridge_log:
        server = subprocess.Popen(
            [
                str(args.server_bin.resolve()),
                "--cad-assets",
                str(root / "field-2027"),
                "--no-referee",
                "--no-lobby",
                "--listen",
                "127.0.0.1:0",
                "--http",
                host.removeprefix("http://"),
            ],
            cwd=root,
            stdout=server_log,
            stderr=subprocess.STDOUT,
        )
        bridge = None
        node = None
        try:
            deadline = time.monotonic() + 45
            while True:
                try:
                    request(host, "/api/state")
                    break
                except OSError:
                    if server.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("CAD server failed to start")
                    time.sleep(0.1)
            request(
                host,
                "/api/command",
                {
                    "SpawnSentry": {
                        "team": "Red",
                        "position_m": [4.0, -3.0, 1.0],
                        "yaw_deg": 90.0,
                    }
                },
            )
            state = request(host, "/api/state")
            chassis = state["field"]["chassis"][0]["id"]
            rclpy.init(args=[])
            node = rclpy.create_node("rm_external_control_smoke")
            stamps = []
            node.create_subscription(
                Clock,
                "/clock",
                lambda msg: stamps.append(msg.clock),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            publisher = node.create_publisher(
                TwistStamped, "/test_nav/cmd_vel", qos_profile_sensor_data
            )
            start = state["field"]["chassis"][0]["pose"]["translation_m"]
            for lease, mode in enumerate(("body", "world", "heading"), 1):
                stamps.clear()
                stage_time_ns = request(host, "/api/state")["field"]["time_ns"]
                bridge = subprocess.Popen(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("control.py")),
                        "--host",
                        host,
                        "--chassis",
                        str(chassis),
                        "--lease-id",
                        str(lease),
                        "--mode",
                        mode,
                        "--topic",
                        "/test_nav/cmd_vel",
                        "--publish-clock",
                    ],
                    stdout=bridge_log,
                    stderr=subprocess.STDOUT,
                )
                deadline = time.monotonic() + 8
                while (
                    not stamps
                    or stamps[-1].sec * 1_000_000_000 + stamps[-1].nanosec
                    < stage_time_ns
                    or publisher.get_subscription_count() == 0
                ):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    assert bridge.poll() is None, "bridge exited before discovery"
                    assert time.monotonic() < deadline, "DDS discovery timeout"
                deadline = time.monotonic() + 1.2
                observed = False
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.01)
                    msg = TwistStamped()
                    msg.header.stamp = stamps[-1]
                    msg.header.frame_id = "base_link" if mode == "body" else "map"
                    msg.twist.linear.x = 0.3
                    publisher.publish(msg)
                    time.sleep(0.04)
                    body = request(host, "/api/state")["field"]["chassis"][0]
                    command = body["command"]
                    observed |= (
                        math.hypot(command["forward_m_s"], command["left_m_s"]) > 0.29
                    )
                assert observed, f"{mode}: no nonzero command reached the live host"
                if mode == "heading":
                    # An accepted manual command revokes the current lease. The
                    # still-running bridge must not reclaim it with a new sequence.
                    request(
                        host,
                        "/api/command",
                        {
                            "Chassis": {
                                "chassis": chassis,
                                "command": {
                                    "forward_m_s": 0.0,
                                    "left_m_s": 0.0,
                                    "yaw_rate_rad_s": 0.0,
                                },
                            }
                        },
                    )
                    publisher.publish(msg)
                else:
                    # Stop publishing commands but leave the bridge's timer alive.
                    # It must not keep refreshing the previous command.
                    pass
                time.sleep(0.4)
                body = request(host, "/api/state")["field"]["chassis"][0]
                command = body["command"]
                assert all(
                    command[k] == 0
                    for k in ("forward_m_s", "left_m_s", "yaw_rate_rad_s")
                ), mode
                stop(bridge)
                assert bridge.returncode == 0, "bridge did not shut down cleanly"
                bridge = None
                print(
                    f"{mode}: live command delivery and {'manual takeover' if mode == 'heading' else 'stale-command braking'} passed"
                )
            end = body["pose"]["translation_m"]
            displacement = math.dist(start[:2], end[:2])
            assert (
                displacement > 0.05
            ), f"commands did not physically move the chassis: {start} -> {end}"
            print(f"Live CAD robot displacement: {displacement:.3f} m")
        except BaseException:
            for label, log in (("server", server_log), ("bridge", bridge_log)):
                log.seek(0)
                print(f"{label} log:\n{log.read()}", file=sys.stderr)
            raise
        finally:
            if bridge is not None:
                stop(bridge)
            stop(server)
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
