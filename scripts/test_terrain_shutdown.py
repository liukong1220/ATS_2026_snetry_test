#!/usr/bin/env python3
"""Exercise installed terrain binaries through real SIGINT shutdown.

Source the ROS/workspace setup first and use an isolated ROS_DOMAIN_ID. This is
an integration regression, not a source-text check. Repeated phase offsets probe
the 100 Hz loop; they do not deterministically force every shutdown race.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rcl_interfaces.srv import GetParameters


def check_shutdown(observer, package, executable, iteration):
    binary = Path(get_package_prefix(package)) / "lib" / package / executable
    node_name = f"terrain_shutdown_{os.getpid()}_{iteration}"
    client = observer.create_client(GetParameters, f"/{node_name}/get_parameters")
    process = None
    with tempfile.TemporaryFile(mode="w+") as output:
        try:
            process = subprocess.Popen(
                [str(binary), "--ros-args", "-r", f"__node:={node_name}"],
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + 15.0
            while not client.wait_for_service(timeout_sec=0.1):
                if process.poll() is not None:
                    raise AssertionError(f"exited during startup: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise AssertionError("parameter service did not become ready")

            # A response proves spin_some is servicing the node, not merely that
            # DDS discovered entities while its constructor was still running.
            request = GetParameters.Request()
            request.names = ["use_sim_time"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(observer, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                raise AssertionError("node did not service its readiness request")
            time.sleep((iteration % 10) * 0.001)
            if process.poll() is not None:
                raise AssertionError(f"exited before SIGINT: {process.returncode}")
            process.send_signal(signal.SIGINT)
            try:
                returncode = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired as exc:
                raise AssertionError("node did not exit within 5 seconds of SIGINT") from exc
            if returncode != 0:
                raise AssertionError(f"SIGINT shutdown exit code was {returncode}, expected 0")
        except Exception:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            output.seek(0)
            print(f"FAIL: {package}/{executable}, iteration {iteration}")
            print(output.read(), flush=True)
            raise
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            observer.destroy_client(client)
    print(f"PASS: {package}/{executable}, iteration {iteration}: SIGINT -> exit 0")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=20)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be positive")
    rclpy.init(args=[])
    observer = rclpy.create_node(f"terrain_shutdown_observer_{os.getpid()}")
    try:
        for package, executable in (
            ("terrain_analysis", "terrainAnalysis"),
            ("terrain_analysis_ext", "terrainAnalysisExt"),
        ):
            for iteration in range(args.iterations):
                check_shutdown(observer, package, executable, iteration)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
