#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Bridge stamped ROS velocity commands into an existing game's operator HTTP API.

This low-rate prototype owns no world. One pending command is retained, and each
ROS message is submitted once; timers never renew a stale command's lease.
"""
import argparse
import json
import math
import time
import urllib.error
import urllib.request


def motion(mode, x, y, yaw_rate):
    """Convert ROS planar components into the server's explicit motion modes."""
    if not all(math.isfinite(v) for v in (x, y, yaw_rate)):
        raise ValueError("nonfinite velocity")
    if math.hypot(x, y) > 6 or abs(yaw_rate) > 4:
        raise ValueError("velocity exceeds adapter limits")
    if mode == "heading" and yaw_rate != 0:
        raise ValueError("heading mode derives yaw rate; angular.z must be zero")
    if mode == "heading" and (x != 0 or y != 0):
        return {
            "HeadingVector": {"direction_world": [x, y], "speed_m_s": math.hypot(x, y)}
        }
    kind = "BodyVelocity" if mode == "body" else "WorldVelocity"
    return {kind: {"velocity_m_s": [x, y], "yaw_rate_rad_s": yaw_rate}}


def request(base, path, payload=None):
    """One bounded HTTP operation; do not blindly retry rejected motion."""
    data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
    req = urllib.request.Request(
        base + path, data=data, headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=0.2) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise OSError(
            f"HTTP {error.code}: {error.read().decode('utf-8', errors='replace')}"
        ) from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="http://127.0.0.1:7780")
    parser.add_argument("--chassis", type=int, required=True)
    parser.add_argument(
        "--lease-id",
        type=int,
        default=time.time_ns(),
        help="Must exceed previous lease for this chassis",
    )
    parser.add_argument(
        "--mode", choices=["body", "world", "heading"], default="heading"
    )
    parser.add_argument("--topic", default="/sentry/cmd_vel")
    parser.add_argument(
        "--frame-id", help="Expected body or world frame; default base_link or map"
    )
    parser.add_argument(
        "--publish-clock",
        action="store_true",
        help="Become the single match /clock publisher",
    )
    args = parser.parse_args()
    if not 0 < args.lease_id < 2**64 or args.chassis < 0:
        parser.error(
            "chassis must be nonnegative and lease-id must be a nonzero uint64"
        )
    frame_id = args.frame_id or ("base_link" if args.mode == "body" else "map")

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[])
    node = rclpy.create_node("rm_simulator_control")
    clock = (
        node.create_publisher(Clock, "/clock", qos_profile_sensor_data)
        if args.publish_clock
        else None
    )
    initial = request(args.host, "/api/state")
    body = next(
        (b for b in initial["field"]["chassis"] if b["id"] == args.chassis), None
    )
    if body is None or initial["paused"]:
        node.destroy_node()
        rclpy.try_shutdown()
        raise RuntimeError("select an existing chassis in an unpaused match")
    epoch = initial["input_epoch"]
    revision = body["placement_revision"]
    pending = None
    sequence = 0
    armed = True

    def receive(msg):
        nonlocal pending, sequence
        if not armed:
            return
        try:
            if msg.header.frame_id != frame_id:
                raise ValueError(f"expected command frame {frame_id!r}")
            if any(
                v != 0
                for v in (msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y)
            ):
                raise ValueError(
                    "only planar x/y velocity and z yaw rate are supported"
                )
            command = motion(
                args.mode, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z
            )
            sequence += 1
            pending = {
                "ExternalDrive": {
                    "chassis": args.chassis,
                    "input": {
                        "lease_id": args.lease_id,
                        "sequence": sequence,
                        "sampled_time_ns": msg.header.stamp.sec * 1_000_000_000
                        + msg.header.stamp.nanosec,
                        "input_epoch": epoch,
                        "placement_revision": revision,
                        "motion": command,
                    },
                }
            }
        except ValueError as error:
            node.get_logger().warning(str(error))

    def pump():
        nonlocal pending, armed
        if not armed and clock is None:
            return
        try:
            # Polling full state is a prototype transport, not high-rate sensor export.
            if clock is not None:
                state = request(args.host, "/api/state")
                stamp = state["field"]["time_ns"]
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = divmod(stamp, 1_000_000_000)
                clock.publish(msg)
            if armed and pending is not None:
                command, pending = pending, None
                request(args.host, "/api/command", command)
        except (OSError, ValueError) as error:
            was_armed, armed = armed, False
            pending = None
            if was_armed:
                node.get_logger().error(
                    f"Control disarmed: {error}; restart with a newer lease after checking the game"
                )

    node.create_subscription(TwistStamped, args.topic, receive, qos_profile_sensor_data)
    node.create_timer(0.05, pump)
    node.get_logger().info(
        f"{args.mode} control: chassis {args.chassis}, lease {args.lease_id}, frame {frame_id}"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            request(
                args.host,
                "/api/command",
                {
                    "ReleaseExternalDrive": {
                        "chassis": args.chassis,
                        "lease_id": args.lease_id,
                    }
                },
            )
        except (OSError, ValueError):
            pass  # Host timeout remains authoritative if release cannot be delivered.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
