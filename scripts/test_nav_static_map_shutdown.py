#!/usr/bin/env python3
"""Check installed nav map publisher SIGINT after receiving its real map.

Source ROS and workspace setup first. Reserve two unused consecutive domains
with --domain-base; no simulator, runner, or production map is required.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def check_shutdown(binary, map_yaml, domain, delivery):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    observer = rclpy.create_node(
        f"nav_static_map_shutdown_observer_{os.getpid()}", context=context
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    received = []
    subscription = observer.create_subscription(
        OccupancyGrid,
        "/map",
        received.append,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    process = None
    with tempfile.TemporaryFile(mode="w+") as output:
        try:
            process = subprocess.Popen(
                [str(binary), "--ros-args", "-p", f"map_yaml_file:={map_yaml}"],
                env={**os.environ, "ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"},
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + 15.0
            while not received:
                if process.poll() is not None:
                    raise AssertionError(f"publisher exited during startup: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise AssertionError("no map received within 15 seconds")
                executor.spin_once(timeout_sec=0.1)
            grid = received[0]
            if (grid.header.frame_id, grid.info.width, grid.info.height, list(grid.data)) != (
                "map", 2, 2, [0, 100, 100, 0]
            ):
                raise AssertionError("received map does not match the deterministic fixture")
            if process.poll() is not None:
                raise AssertionError(f"publisher exited before SIGINT: {process.returncode}")
            if delivery == "owner":
                process.send_signal(signal.SIGINT)
            else:
                # Only this child owns this fresh session/process group. Never
                # signal the observer's group or any existing navigation runner.
                os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=5.0)
            output.seek(0)
            log = output.read()
            if returncode != 0:
                raise AssertionError(f"SIGINT exit code {returncode}, expected 0")
            if "Traceback" in log or "rcl_shutdown already called" in log:
                raise AssertionError("shutdown emitted a traceback or duplicate shutdown error")
        except Exception:
            output.seek(0)
            print(f"FAIL: delivery={delivery}, domain={domain}\n{output.read()}", flush=True)
            raise
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)
            observer.destroy_subscription(subscription)
            executor.shutdown()
            observer.destroy_node()
            context.try_shutdown()
    print(f"PASS: delivery={delivery}, domain={domain}: map received; SIGINT -> exit 0")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-base", type=int, required=True,
                        help="first of two reserved unused ROS domains (0..231)")
    args = parser.parse_args()
    if not 0 <= args.domain_base <= 231:
        parser.error("--domain-base must be in 0..231")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    package = "ats_nav_bringup"
    binary = Path(get_package_prefix(package)) / "lib" / package / "static_map_publisher.py"
    if not binary.is_file():
        raise FileNotFoundError(f"build and source {package} first: {binary}")
    with tempfile.TemporaryDirectory(prefix="nav_static_map_shutdown_") as directory:
        fixture = Path(directory)
        (fixture / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 0]))
        map_yaml = fixture / "map.yaml"
        map_yaml.write_text(
            "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        for offset, delivery in enumerate(("owner", "process_group")):
            check_shutdown(binary, map_yaml, args.domain_base + offset, delivery)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
