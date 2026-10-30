#!/usr/bin/env python3
"""Replay an S1 regression bag through Point-LIO alone and compare runs bit for bit (report A.20).

run      start pointlio_mapping with node_params.yaml (use_sim_time, gravity of the chassis mount as
         lio_launch.py computes it, optional --set overrides), publish /clock /livox/lidar
         /livox/imu of BAG in lockstep (see lockstep_play) while a sink node spun in its own
         thread keeps every serialized /aft_mapped_to_init and /cloud_registered, sample the
         point_lio process from /proc, then reduce to OUT/summary.json: one entry per
         /cloud_registered frame with the cloud digest (header, layout and declared fields; the
         PointXYZINormal padding is uninitialised) and the frame-end pose (the last
         /aft_mapped_to_init whose stamp is not after the cloud stamp) as float.hex(), plus a
         digest over each whole stream.
compare  read two summary.json files and report whether frame-end poses and clouds are identical;
         exit 0 only when both are bit-exact (--clouds-only: when the clouds are).

    ROS_DOMAIN_ID=131 python3 scripts/lio_bag_replay.py run \
        --bag ~/ats_evidence/s1_round4/bags/static_bag --out ~/ats_evidence/s1_round4/a20/base/static
    python3 scripts/lio_bag_replay.py compare A/summary.json B/summary.json

Why lockstep instead of ``ros2 bag play --rate 0.5`` (report A.20 step 2): Point-LIO subscribes
/livox/* with SensorDataQoS (best effort, depth 5) and takes no message while a frame is being
processed (the 500 Hz polling loop of that time took at most one per subscription per 2 ms
iteration; S2a item 3 replaced it with spin_some + spin_once).  Two ``--rate 0.5`` replays
of the same bag on the same binary differed from the first frame on (IMU initialisation counted 9 %
vs 8 % after the second frame), i.e. samples were dropped.  ``ros2 bag record`` lost 4 of 373
clouds, hence the in-process sink.  /aft_mapped_to_init with publish_odometry_without_downsample
True comes in bursts of hundreds of messages per frame through a depth-20 KEEP_LAST publisher
and loses messages even to a reliable subscriber (two replays of tunnel_entry: 120561 vs 108922
messages, 29 of 373 frame ends on different points, clouds identical), so in that mode only the
clouds are bit-exact evidence (compare --clouds-only); with False every frame publishes one
odometry and the frame-end poses are compared too.
"""

import argparse
import bisect
import hashlib
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
# lio_launch.py: mount_pose_from_configurations(...).gravity_in_sensor() for the default chassis
# mount (x 0.2405 m, 0.20 m above ground, front tilt 30 deg, base_footprint_height 0.18 m).
DEFAULT_GRAVITY = (4.905, 0.0, -8.495709)
INPUT_TOPICS = ("/clock", "/livox/lidar", "/livox/imu")
ODOM, CLOUD = "/aft_mapped_to_init", "/cloud_registered"
CLK_TCK = os.sysconf("SC_CLK_TCK")


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _hex_pose(pose):
    return [float(v).hex() for v in pose]


def frame_end_poses(odoms, clouds):
    """odoms: [(stamp_ns, (px, py, pz, qx, qy, qz, qw))], clouds: [(stamp_ns, sha256)], both in
    publication order.  Each cloud frame gets the last odometry whose stamp is not after the cloud
    stamp and after the previous cloud stamp (Point-LIO stamps per-point odometry with the point
    time and the cloud with lidar_end_time), or None when the frame published no odometry."""
    stamps = [stamp for stamp, _ in odoms]
    frames, previous = [], None
    for cloud_stamp, cloud_sha in clouds:
        end = bisect.bisect_right(stamps, cloud_stamp)
        begin = 0 if previous is None else bisect.bisect_right(stamps, previous)
        entry = {"cloud_stamp_ns": cloud_stamp, "cloud_sha256": cloud_sha,
                 "odom_in_frame": max(0, end - begin), "odom_stamp_ns": None, "pose_hex": None}
        if end > begin:
            stamp, pose = odoms[end - 1]
            entry["odom_stamp_ns"] = stamp
            entry["pose_hex"] = _hex_pose(pose)
        frames.append(entry)
        previous = cloud_stamp
    return frames


def compare_summaries(a, b):
    """Bit-exact comparison of two run summaries, frame by frame in order."""
    fa, fb = a["frames"], b["frames"]
    result = {"frames_a": len(fa), "frames_b": len(fb), "cloud_mismatch": 0, "pose_mismatch": 0,
              "cloud_stamp_mismatch": 0, "first_mismatch_frame": None,
              "max_abs_position_m": 0.0, "max_abs_quaternion": 0.0,
              "odom_stream_identical": a["odom"]["sha256"] == b["odom"]["sha256"],
              "cloud_stream_identical": a["cloud"]["sha256"] == b["cloud"]["sha256"]}
    for index, (x, y) in enumerate(zip(fa, fb)):
        bad = False
        if x["cloud_stamp_ns"] != y["cloud_stamp_ns"]:
            result["cloud_stamp_mismatch"] += 1
            bad = True
        if x["cloud_sha256"] != y["cloud_sha256"]:
            result["cloud_mismatch"] += 1
            bad = True
        if x["pose_hex"] != y["pose_hex"]:
            result["pose_mismatch"] += 1
            bad = True
            if x["pose_hex"] and y["pose_hex"]:
                px = [float.fromhex(v) for v in x["pose_hex"]]
                py = [float.fromhex(v) for v in y["pose_hex"]]
                result["max_abs_position_m"] = max(
                    result["max_abs_position_m"], *(abs(p - q) for p, q in zip(px[:3], py[:3])))
                # q and -q are the same rotation; compare against the closer sign.
                sign = 1.0 if sum(p * q for p, q in zip(px[3:], py[3:])) >= 0.0 else -1.0
                result["max_abs_quaternion"] = max(
                    result["max_abs_quaternion"],
                    *(abs(p - sign * q) for p, q in zip(px[3:], py[3:])))
        if bad and result["first_mismatch_frame"] is None:
            result["first_mismatch_frame"] = index
    result["clouds_bit_exact"] = (len(fa) == len(fb) and result["cloud_mismatch"] == 0
                                  and result["cloud_stamp_mismatch"] == 0)
    result["bit_exact"] = result["clouds_bit_exact"] and result["pose_mismatch"] == 0
    return result


class ProcSampler:
    """CPU and memory of one pid from /proc, sampled by the caller once per second."""

    def __init__(self, pid):
        self.pid = pid
        self.cpu_pct, self.rss_mib = [], []
        self.start_ticks = self.last_ticks = self._ticks()
        self.start_wall = self.last_wall = time.monotonic()

    def _ticks(self):
        with open(f"/proc/{self.pid}/stat", encoding="ascii") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])   # utime + stime (fields 14, 15 of stat)

    def _status_kib(self, key):
        with open(f"/proc/{self.pid}/status", encoding="ascii") as stream:
            for line in stream:
                if line.startswith(key + ":"):
                    return int(line.split()[1])
        return 0

    def sample(self):
        ticks, wall = self._ticks(), time.monotonic()
        if wall > self.last_wall:
            self.cpu_pct.append(100.0 * (ticks - self.last_ticks) / CLK_TCK / (wall - self.last_wall))
        self.rss_mib.append(self._status_kib("VmRSS") / 1024.0)
        self.last_ticks, self.last_wall = ticks, wall
        return self.cpu_pct[-1] if self.cpu_pct else 0.0

    def summary(self):
        cpu = self.cpu_pct or [0.0]
        rss = self.rss_mib or [0.0]
        return {"cpu_seconds": (self.last_ticks - self.start_ticks) / CLK_TCK,
                "wall_seconds": self.last_wall - self.start_wall,
                "cpu_mean_pct": sum(cpu) / len(cpu), "cpu_peak_pct": max(cpu),
                "rss_mean_mib": sum(rss) / len(rss), "rss_peak_mib": max(rss),
                "vm_hwm_mib": self._status_kib("VmHWM") / 1024.0, "samples": len(self.cpu_pct)}


def cloud_digest(msg):
    """SHA-256 of a PointCloud2 over its header, layout and declared fields only.  PCL's
    PointXYZINormal carries 16 padding bytes per point that toROSMsg copies uninitialised, so the
    raw bytes differ between identical runs."""
    import numpy as np

    digest = hashlib.sha256()
    digest.update(struct.pack("<iI", msg.header.stamp.sec, msg.header.stamp.nanosec))
    digest.update(msg.header.frame_id.encode())
    digest.update(struct.pack("<II?", msg.height, msg.width, msg.is_dense))
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    points = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height * msg.width, msg.point_step)
    for field in sorted(msg.fields, key=lambda f: f.offset):
        width = sizes[field.datatype] * field.count
        digest.update(f"{field.name}:{field.offset}:{field.datatype}:{field.count};".encode())
        digest.update(np.ascontiguousarray(points[:, field.offset:field.offset + width]).tobytes())
    return digest.hexdigest()


def reduce_messages(odom_raw, cloud_raw):
    """Frame-end poses, cloud digests and stream digests of the recorded serialized messages."""
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    odoms, clouds = [], []
    odom_stream, cloud_stream = hashlib.sha256(), hashlib.sha256()
    for data in odom_raw:
        msg = deserialize_message(data, Odometry)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        odom_stream.update(struct.pack("<q7d", stamp, *pose))
        odoms.append((stamp, pose))
    for data in cloud_raw:
        msg = deserialize_message(data, PointCloud2)
        sha = cloud_digest(msg)
        cloud_stream.update(sha.encode())
        clouds.append((msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, sha))
    inversions = sum(1 for a, b in zip(odoms, odoms[1:]) if b[0] < a[0])
    # Stable sort: messages with equal stamps keep their publication order.
    ordered = sorted(odoms, key=lambda entry: entry[0]) if inversions else odoms
    return {"odom": {"count": len(odoms), "sha256": odom_stream.hexdigest(),
                     "stamp_inversions": inversions},
            "cloud": {"count": len(clouds), "sha256": cloud_stream.hexdigest()},
            "frames": frame_end_poses(ordered, clouds)}


def _stop(process, timeout):
    if process is None or process.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


def lidar_frame_end_ns(data):
    """Header stamp + largest point offset_time of a serialized livox_ros_driver2/CustomMsg, in ns:
    an upper bound of the lidar_end_time Point-LIO derives from the points it keeps.  Layout after
    the 4-byte CDR encapsulation: stamp (int32, uint32), frame_id (uint32 length + bytes), pad to 8,
    timebase uint64, point_num uint32, lidar_id uint8, rsvd uint8[3], points (uint32 count, then
    20-byte CustomPoint with offset_time first)."""
    import numpy as np

    body = memoryview(data)[4:]
    sec, nsec, length = struct.unpack_from("<iII", body, 0)
    pos = (12 + length + 7) & ~7
    count = struct.unpack_from("<I", body, pos + 16)[0]
    offsets = np.ndarray((count,), dtype="<u4", buffer=body, offset=pos + 20, strides=(20,))
    return sec * 1_000_000_000 + nsec + (int(offsets.max()) if count else 0)


def header_stamp_ns(data):
    sec, nsec = struct.unpack_from("<iI", data, 4)
    return sec * 1_000_000_000 + nsec


def lockstep_play(bag, args, sampler):
    """Publish the bag's /clock, /livox/imu and /livox/lidar in recorded order.  Point-LIO starts a
    frame as soon as it holds the LiDAR message and an IMU sample at or after the frame end
    (sync_packages), and does not take messages while it works on it.  So once both have been
    published, nothing more is sent until that frame's /cloud_registered arrives (or
    --frame-timeout-s passes, e.g. during IMU initialisation, which publishes no cloud), and IMU
    samples are spaced by --imu-gap-s: Point-LIO's depth-5 best-effort queues never overflow.
    Gating only before the next LiDAR message still dropped IMU samples published while a frame
    was being processed (two runs of spin_walk differed from the second initialisation frame).
    Returns the replay statistics and the serialized odometry and cloud messages received."""
    import rclpy
    import rosbag2_py
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2

    rclpy.init()
    node = rclpy.create_node("lio_bag_replay_player")
    sink = rclpy.create_node("lio_bag_replay_sink")
    executor = SingleThreadedExecutor()
    executor.add_node(sink)
    stats = {"lidar": 0, "imu": 0, "clock": 0, "clouds": 0, "frame_timeouts": 0,
             "frame_timeouts_after_first_cloud": 0, "wall_s": 0.0}
    odom_raw, cloud_raw = [], []

    def on_cloud(data):
        cloud_raw.append(bytes(data))
        stats["clouds"] = len(cloud_raw)

    reliable = ReliabilityPolicy.RELIABLE
    sink.create_subscription(PointCloud2, CLOUD, on_cloud, QoSProfile(depth=50, reliability=reliable),
                             raw=True)
    sink.create_subscription(Odometry, ODOM, lambda data: odom_raw.append(bytes(data)),
                             QoSProfile(depth=5000, reliability=reliable), raw=True)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pubs = {"/livox/lidar": node.create_publisher(CustomMsg, "/livox/lidar", qos),
                "/livox/imu": node.create_publisher(Imu, "/livox/imu", qos),
                "/clock": node.create_publisher(Clock, "/clock", qos)}
        deadline = time.monotonic() + 60.0
        while not (pubs["/livox/lidar"].get_subscription_count() >= 1
                   and pubs["/livox/imu"].get_subscription_count() >= 1
                   and node.count_publishers(ODOM) >= 1 and node.count_publishers(CLOUD) >= 1):
            if time.monotonic() > deadline:
                raise RuntimeError("point_lio did not connect within 60 s")
            time.sleep(0.2)
        time.sleep(1.0)   # let the remaining endpoint matches settle before the first sample

        def wait_for_cloud(clouds_before):
            until = time.monotonic() + args.frame_timeout_s
            while stats["clouds"] <= clouds_before and time.monotonic() < until:
                time.sleep(0.001)
            if stats["clouds"] <= clouds_before:
                stats["frame_timeouts"] += 1
                if clouds_before > 0:
                    stats["frame_timeouts_after_first_cloud"] += 1

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(INPUT_TOPICS)))
        # Frame ends (ns) of LiDAR messages published but not yet started by Point-LIO, oldest first.
        pending, last_imu_ns = [], None
        start, last_sample = time.monotonic(), time.monotonic()
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == "/livox/lidar":
                pending.append(lidar_frame_end_ns(data))
                stats["lidar"] += 1
            elif topic == "/livox/imu":
                last_imu_ns = header_stamp_ns(data)
                stats["imu"] += 1
                time.sleep(args.imu_gap_s)
            else:
                stats["clock"] += 1
            clouds_before = stats["clouds"]
            pubs[topic].publish(bytes(data))
            # 1 us margin: lidar_end_time comes from float32 point times and may round up past
            # an IMU stamp equal to the frame end; then Point-LIO waits for the next sample.
            while pending and last_imu_ns is not None and last_imu_ns >= pending[0] + 1000:
                pending.pop(0)
                wait_for_cloud(clouds_before)
                clouds_before = stats["clouds"]
            if time.monotonic() - last_sample >= 1.0:
                sampler.sample()
                last_sample = time.monotonic()
        time.sleep(1.0)   # late odometry of the last frame may still be in flight
        stats["wall_s"] = time.monotonic() - start
        return stats, list(odom_raw), list(cloud_raw)
    finally:
        executor.shutdown()
        spinner.join(5.0)
        node.destroy_node()
        sink.destroy_node()
        rclpy.shutdown()


def _git_state(path):
    try:
        head = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(path), "status", "--porcelain", "--", "."],
                               capture_output=True, text=True, check=True).stdout.strip()
        return {"head": head, "dirty_files": dirty.splitlines()}
    except (OSError, subprocess.CalledProcessError):
        return None


def run(args):
    if not os.environ.get("ROS_DOMAIN_ID"):
        sys.exit("set ROS_DOMAIN_ID to an isolated domain first")
    bag = Path(args.bag).expanduser().resolve()
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=False)
    exe = Path(args.executable).resolve()
    params = Path(args.params).resolve()
    gravity = "[" + ", ".join(repr(float(v)) for v in args.gravity) + "]"
    log = open(out / "point_lio.log", "w", encoding="utf-8")
    lio = None
    try:
        lio = subprocess.Popen(
            [str(exe), "--ros-args", "-r", "__node:=point_lio", "--params-file", str(params),
             "-p", "use_sim_time:=true", "-p", f"mapping.gravity:={gravity}",
             "-p", f"mapping.gravity_init:={gravity}"]
            + [item for override in args.set for item in ("-p", override)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(1.0)
        sampler = ProcSampler(lio.pid)
        replay, odom_raw, cloud_raw = lockstep_play(bag, args, sampler)
        # Point-LIO may still be working through its buffers; wait until it idles.
        idle, deadline = 0, time.monotonic() + args.drain_s
        while idle < 3 and time.monotonic() < deadline and lio.poll() is None:
            time.sleep(1.0)
            idle = idle + 1 if sampler.sample() < args.idle_pct else 0
        cpu = sampler.summary()
        if lio.poll() is not None:
            raise RuntimeError(f"point_lio exited early with {lio.returncode}")
        _stop(lio, 10.0)
    finally:
        _stop(lio, 10.0)
        log.close()
    summary = {"schema": 1, "created_unix": time.time(), "bag": str(bag),
               "bag_metadata_sha256": _sha256_file(bag / "metadata.yaml"),
               "imu_gap_s": args.imu_gap_s, "frame_timeout_s": args.frame_timeout_s,
               "domain": os.environ["ROS_DOMAIN_ID"], "executable": str(exe),
               "executable_sha256": _sha256_file(exe), "params": str(params),
               "params_sha256": _sha256_file(params), "gravity": list(args.gravity),
               "overrides": list(args.set),
               "point_lio_git": _git_state(WS / "src/ats_sentry_nav/point_lio"),
               "replay": replay, "point_lio_process": cpu}
    summary.update(reduce_messages(odom_raw, cloud_raw))
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(f"{bag.name}: frames {len(summary['frames'])}  odom {summary['odom']['count']}  "
          f"timeouts {replay['frame_timeouts']} (after first cloud "
          f"{replay['frame_timeouts_after_first_cloud']})  wall {replay['wall_s']:.0f} s  "
          f"cloud {summary['cloud']['count']}  point_lio cpu {cpu['cpu_seconds']:.1f} s "
          f"(mean {cpu['cpu_mean_pct']:.1f} %, peak {cpu['cpu_peak_pct']:.1f} %)  "
          f"rss peak {cpu['rss_peak_mib']:.1f} MiB  hwm {cpu['vm_hwm_mib']:.1f} MiB")
    return 0 if summary["frames"] else 1


def compare(args):
    a = json.loads(Path(args.a).read_text(encoding="utf-8"))
    b = json.loads(Path(args.b).read_text(encoding="utf-8"))
    result = compare_summaries(a, b)
    for side, summary in (("a", a), ("b", b)):
        cpu = summary["point_lio_process"]
        result[f"cpu_seconds_{side}"] = cpu["cpu_seconds"]
        result[f"vm_hwm_mib_{side}"] = cpu["vm_hwm_mib"]
    print(json.dumps(result, indent=1))
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=1), encoding="utf-8")
    return 0 if result["clouds_bit_exact" if args.clouds_only else "bit_exact"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p_run = sub.add_parser("run", help="replay one bag through point_lio and summarize the output")
    p_run.add_argument("--bag", required=True)
    p_run.add_argument("--out", required=True, help="new directory for logs and summary.json")
    p_run.add_argument("--imu-gap-s", type=float, default=0.003,
                       help="wall pause before each IMU sample (Point-LIO takes one per 2 ms loop)")
    p_run.add_argument("--frame-timeout-s", type=float, default=1.0)
    p_run.add_argument("--set", action="append", default=[], metavar="NAME:=VALUE",
                       help="extra point_lio parameter override, e.g. "
                            "odometry.publish_odometry_without_downsample:=false")
    p_run.add_argument("--executable", default=str(WS / "install/point_lio/lib/point_lio/pointlio_mapping"))
    p_run.add_argument("--params", default=str(
        WS / "install/ats_sentry_bringup/share/ats_sentry_bringup/params/node_params.yaml"))
    p_run.add_argument("--gravity", type=float, nargs=3, default=DEFAULT_GRAVITY)
    p_run.add_argument("--drain-s", type=float, default=60.0)
    p_run.add_argument("--idle-pct", type=float, default=15.0)
    p_run.set_defaults(func=run)
    p_cmp = sub.add_parser("compare", help="bit-exact comparison of two summary.json files")
    p_cmp.add_argument("a")
    p_cmp.add_argument("b")
    p_cmp.add_argument("--out")
    p_cmp.add_argument("--clouds-only", action="store_true",
                       help="exit 0 when the clouds alone are bit-exact (per-point odometry is lossy)")
    p_cmp.set_defaults(func=compare)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
