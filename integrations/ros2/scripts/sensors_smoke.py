#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check synchronized live CAD host -> HTTP -> rclrs -> ROS measured sensors."""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import time
from control import request


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-bin", type=Path, default=root / "target/nix/debug/rm-simulator-server")
    parser.add_argument("--cad-assets", type=Path, default=root / "field-2027")
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--moving", action="store_true")
    parser.add_argument("--rmw", default="rmw_cyclonedds_cpp")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        # Cover native node teardown as well as host startup and bridge delivery.
        with tempfile.TemporaryFile(mode="w+") as output:
            worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                worker.wait(timeout=100)
            except subprocess.TimeoutExpired:
                os.killpg(worker.pid, signal.SIGKILL)
                worker.wait()
                output.seek(0)
                print(output.read(), end="")
                raise SystemExit("FAIL: synchronized sensor lifecycle exceeded 100 s")
            finally:
                try:
                    os.killpg(worker.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            output.seek(0)
            print(output.read(), end="")
            raise SystemExit(worker.returncode)
    os.environ["RMW_IMPLEMENTATION"] = args.rmw
    os.environ["ROS_DOMAIN_ID"] = "76"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
    os.environ.setdefault("CYCLONEDDS_URI", "file://" + str(root / "integrations/ros2/config/cyclonedds-loopback.xml"))
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{sock.getsockname()[1]}"
    url = "http://" + address
    with tempfile.TemporaryFile(mode="w+") as server_log, tempfile.TemporaryFile(mode="w+") as bridge_log:
        children = []
        node = None
        try:
            server = subprocess.Popen([str(args.server_bin.resolve()), "--cad-assets", str(args.cad_assets.resolve()), "--no-referee", "--no-lobby", "--listen", "127.0.0.1:0", "--http", address], cwd=root, stdout=server_log, stderr=subprocess.STDOUT)
            children.append(server)
            deadline = time.monotonic() + 45
            while True:
                try:
                    state = request(url, "/api/state")
                    break
                except OSError:
                    if server.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("sensor host failed to start")
                    time.sleep(.1)
            if args.moving or not state["field"]["chassis"]:
                request(url, "/api/command", {"SpawnSentry": {"team": "Red", "position_m": [4., -3., 1.], "yaw_deg": 0.}})
                state = request(url, "/api/state")
            robot = max(c["id"] for c in state["field"]["chassis"])
            if args.moving:
                request(url, "/api/command", {"StopSentry": {"chassis": robot}})
                request(url, "/api/command", {"Chassis": {"chassis": robot, "command": {"forward_m_s": 0., "left_m_s": 0., "yaw_rate_rad_s": .5}}})
            rclpy.init(args=[])
            node = rclpy.create_node("rm_synchronized_sensor_smoke")
            scans, inertial, clocks = [], [], []
            qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(CustomMsg, "/sensors_test/lidar", scans.append, qos)
            node.create_subscription(Imu, "/sensors_test/imu", inertial.append, qos)
            node.create_subscription(Clock, "/clock", clocks.append, qos)
            bridge = subprocess.Popen([str(root / "target/ros2/release/rm-simulator-ros2"), "--host", address, "--robot", str(robot), "--frames", "6", "--stride", str(args.stride), "--range-noise-m", "0", "--dropout", "0", "--topic", "/sensors_test/lidar", "--imu-topic", "/sensors_test/imu", "--frame-id", "test_lidar", "--imu-frame-id", "test_imu", "--publish-clock", "--require-subscriber", "--wait-for-subscriber-ms", "5000"], stdout=bridge_log, stderr=subprocess.STDOUT)
            children.append(bridge)
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if bridge.poll() is not None and len(scans) >= 6 and len(inertial) >= 120:
                    break
            assert bridge.poll() == 0, "live bridge failed or hung"
            assert len(scans) == 6, len(scans)
            assert len(inertial) == 120, len(inertial)
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in inertial]
            assert all(b-a == 5_000_000 for a,b in zip(stamps, stamps[1:]))
            for msg in inertial:
                assert msg.header.frame_id == "test_imu"
                assert msg.orientation_covariance[0] == -1.
                assert all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
            force_norms = [math.sqrt(m.linear_acceleration.x**2 + m.linear_acceleration.y**2 + m.linear_acceleration.z**2) for m in inertial]
            assert 3 < statistics.median(force_norms) < 30, "acceleration must use SI units"
            if args.moving:
                assert sum(msg.angular_velocity.z > .1 for msg in inertial) >= 20, (min(m.angular_velocity.z for m in inertial), max(m.angular_velocity.z for m in inertial))
            # Packet first emissions may shift by up to stride*5 us; IMU stays on the packet boundary.
            for n, scan in enumerate(scans):
                assert scan.header.frame_id == "test_lidar"
                assert scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec == scan.timebase
                assert scan.point_num == len(scan.points) > 0
                packet_end = stamps[(n+1)*20-1]
                packet_start = packet_end - 100_000_000
                assert packet_start <= scan.timebase < packet_start + args.stride*5_000
                assert all(scan.timebase+p.offset_time < packet_end for p in scan.points)
            assert clocks and clocks[-1].clock.sec * 10**9 + clocks[-1].clock.nanosec >= stamps[-1]
            print(f"Validated: live CAD host, {len(scans)} rolling scans / {len(inertial)} SI IMU samples; 5 ms cadence, shared clock, offsets and unavailable orientation verified")
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5)
            for log in (server_log, bridge_log):
                log.seek(0)
                print(log.read())
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
        print("PASS: synchronized measurements and clean bridge/subscriber shutdown")


if __name__ == "__main__":
    main()
