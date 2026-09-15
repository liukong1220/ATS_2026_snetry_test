#!/usr/bin/env bash
# Gazebo mapping-time GT localization jump stability (honest DoD).
# Launch GT odometry + correct map->odom seed, GICP off; require no large pose jumps.
set -eo pipefail

WS="${WS:-/home/kong/ATS_2026_snetry_test}"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

DOMAIN="${ROS_DOMAIN_ID:-$((40 + RANDOM % 80))}"
if (( DOMAIN < 0 || DOMAIN > 101 )); then
  echo "ROS_DOMAIN_ID=$DOMAIN out of FastRTPS-safe range [0,101]; clamping" >&2
  DOMAIN=$((40 + RANDOM % 60))
fi
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

RUN_ID="$(date +%Y%m%d_%H%M%S)_domain${DOMAIN}"
OUT="$WS/log/gazebo_mapping_loc/${RUN_ID}"
mkdir -p "$OUT"

TRUTH_X="${TRUTH_X:-1.17}"
TRUTH_Y="${TRUTH_Y:--0.44}"
TRUTH_YAW="${TRUTH_YAW:-0.0}"
export TRUTH_X TRUTH_Y TRUTH_YAW
export WAIT_TOPICS_S="${WAIT_TOPICS_S:-150}"
export SAMPLE_S="${SAMPLE_S:-35}"
export MAX_JUMP_M="${MAX_JUMP_M:-0.15}"
export MAX_LOC_JUMP_M="${MAX_LOC_JUMP_M:-0.30}"
export MAX_DRIFT_M="${MAX_DRIFT_M:-0.50}"

echo "OUT=$OUT" | tee "$OUT/meta.txt"
echo "DOMAIN=$DOMAIN" | tee -a "$OUT/meta.txt"
echo "TRUTH=($TRUTH_X,$TRUTH_Y,$TRUTH_YAW)" | tee -a "$OUT/meta.txt"

pkill -9 -f 'ats_gazebo_nav|ign gazebo|gz sim|gazebo_gt_|localization_fusion|small_gicp|point_lio|rog_map|ros2 launch' 2>/dev/null || true
sleep 2
free -h | head -2 | tee "$OUT/mem_before.txt"

setsid ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  use_sim_time:=true \
  headless:=true \
  headless_rendering:=true \
  use_viewer:=false \
  use_rviz:=false \
  enable_camera_sensors:=false \
  use_gazebo_gt_odometry:=true \
  launch_terrain_analysis:=false \
  launch_small_gicp_relocalization:=false \
  initial_map_to_odom_x:="$TRUTH_X" \
  initial_map_to_odom_y:="$TRUTH_Y" \
  initial_map_to_odom_yaw:="$TRUTH_YAW" \
  >"$OUT/launch.log" 2>&1 &
echo $! >"$OUT/launch.pid"
echo "LAUNCH_PID=$(cat "$OUT/launch.pid")"

python3 - <<'PY' | tee "$OUT/wait_topics.txt"
import os, time, subprocess, sys
need = ["/clock", "/odometry", "/localization"]
deadline = time.time() + float(os.environ.get("WAIT_TOPICS_S", "150"))
seen = {t: False for t in need}
while time.time() < deadline:
    try:
        topics = set(subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=8).splitlines())
    except Exception as exc:
        print("list_fail", exc, flush=True)
        time.sleep(2)
        continue
    for t in need:
        if (not seen[t]) and t in topics:
            seen[t] = True
            print("seen", t, flush=True)
    if all(seen.values()):
        print("ALL_SEEN", flush=True)
        sys.exit(0)
    time.sleep(2)
print("TIMEOUT", seen, file=sys.stderr)
sys.exit(3)
PY

python3 - <<'PY' | tee "$OUT/jump_probe.json"
import json, math, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

truth_x = float(os.environ["TRUTH_X"])
truth_y = float(os.environ["TRUTH_Y"])
truth_yaw = float(os.environ["TRUTH_YAW"])
sample_s = float(os.environ.get("SAMPLE_S", "35"))
max_jump = float(os.environ.get("MAX_JUMP_M", "0.15"))
max_loc_jump = float(os.environ.get("MAX_LOC_JUMP_M", "0.30"))
max_drift = float(os.environ.get("MAX_DRIFT_M", "0.50"))

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class Probe(Node):
    def __init__(self):
        super().__init__("mapping_jump_probe")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.samples = []
        self.create_subscription(Odometry, "/localization", self.on_loc, qos_profile_sensor_data)

    def on_loc(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.samples.append((time.time(), p.x, p.y, yaw_of(q)))

    def map_pose(self):
        for fr in ("gimbal_yaw_odom", "base_link", "base_footprint"):
            try:
                tf = self.tf.lookup_transform("map", fr, Time(), timeout=Duration(seconds=0.05))
                t = tf.transform.translation
                q = tf.transform.rotation
                return t.x, t.y, yaw_of(q), fr
            except Exception:
                continue
        return None

rclpy.init()
node = Probe()
# warmup
t0 = time.time()
while time.time() - t0 < 8.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)

poses = []
jumps = []
prev = None
t0 = time.time()
while time.time() - t0 < sample_s and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)
    cur = node.map_pose()
    if cur is None:
        continue
    poses.append(cur)
    if prev is not None:
        jumps.append(math.hypot(cur[0] - prev[0], cur[1] - prev[1]))
    prev = cur

loc_j = []
for i in range(1, len(node.samples)):
    a = node.samples[i - 1]
    b = node.samples[i]
    loc_j.append(math.hypot(b[1] - a[1], b[2] - a[2]))

last = poses[-1] if poses else None
drift = None if last is None else math.hypot(last[0] - truth_x, last[1] - truth_y)
yaw_err = None if last is None else abs(wrap(last[2] - truth_yaw))
max_j = max(jumps) if jumps else None
max_lj = max(loc_j) if loc_j else None
p95 = sorted(jumps)[int(0.95 * (len(jumps) - 1))] if jumps else None

rep = {
    "n_map_poses": len(poses),
    "n_loc_samples": len(node.samples),
    "max_map_jump_m": max_j,
    "p95_map_jump_m": p95,
    "max_loc_jump_m": max_lj,
    "drift_xy_m": drift,
    "drift_yaw_rad": yaw_err,
    "last_map_pose": None if last is None else {"x": last[0], "y": last[1], "yaw": last[2], "frame": last[3]},
    "thresholds": {
        "max_jump_m": max_jump,
        "max_loc_jump_m": max_loc_jump,
        "max_drift_m": max_drift,
    },
    "pass_jump_stable": bool(
        max_j is not None
        and max_j <= max_jump
        and (max_lj is None or max_lj <= max_loc_jump)
        and drift is not None
        and drift <= max_drift
        and len(poses) >= 10
    ),
}
print(json.dumps(rep, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if rep["pass_jump_stable"] else 4)
PY
PROBE_RC=$?

if [[ -f "$OUT/launch.pid" ]]; then
  kill -- -"$(cat "$OUT/launch.pid")" 2>/dev/null || true
fi
pkill -9 -f 'ats_gazebo_nav|ign gazebo|gz sim|gazebo_gt_|localization_fusion|small_gicp|point_lio|rog_map|ros2 launch' 2>/dev/null || true

echo "PROBE_RC=$PROBE_RC" | tee -a "$OUT/meta.txt"
echo "OUT=$OUT"
exit "$PROBE_RC"
