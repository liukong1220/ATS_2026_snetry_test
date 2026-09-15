#!/usr/bin/env bash
# Gazebo prior-map relocalization (honest DoD).
#
# A) Mapping-time GT localization remains jump-stable (optional preflight).
# B) Prior PCD + WRONG initial map->odom + /initialpose near truth:
#    GICP must publish accepted observations and fusion must recover map pose.
# Does NOT claim red_box 10/10.
set -eo pipefail

WS="${WS:-/home/kong/ATS_2026_snetry_test}"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

DOMAIN="${ROS_DOMAIN_ID:-$((40 + RANDOM % 80))}"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

RUN_ID="$(date +%Y%m%d_%H%M%S)_domain${DOMAIN}"
OUT="$WS/log/gazebo_prior_reloc/${RUN_ID}"
mkdir -p "$OUT"

# Truth map->odom for rmuc_2025 spawn (4.75,9.00) + map origin (-3.58,-9.44).
TRUTH_X="${TRUTH_X:-1.17}"
TRUTH_Y="${TRUTH_Y:--0.44}"
TRUTH_YAW="${TRUTH_YAW:-0.0}"

OFFSET_X="${OFFSET_X:-1.0}"
OFFSET_Y="${OFFSET_Y:-0.8}"
OFFSET_YAW="${OFFSET_YAW:-0.40}"
# Fusion / GICP seed is INTENTIONALLY WRONG; /initialpose carries the truth guess.
WRONG_X="$(python3 -c "print($TRUTH_X + $OFFSET_X)")"
WRONG_Y="$(python3 -c "print($TRUTH_Y + $OFFSET_Y)")"
WRONG_YAW="$(python3 -c "print($TRUTH_YAW + $OFFSET_YAW)")"

PRIOR_PCD="${PRIOR_PCD:-$WS/src/ats_sentry_bringup/pcd/rmuc_2025.pcd}"
if [[ ! -e "$PRIOR_PCD" ]]; then
  echo "MISSING prior PCD: $PRIOR_PCD" | tee "$OUT/error.txt"
  echo "Expected: src/ats_sentry_bringup/pcd/rmuc_2025.pcd" | tee -a "$OUT/error.txt"
  exit 2
fi
PRIOR_PCD="$(readlink -f "$PRIOR_PCD")"

export TRUTH_X TRUTH_Y TRUTH_YAW OFFSET_X OFFSET_Y OFFSET_YAW
export RECOVER_XY_M="${RECOVER_XY_M:-0.60}"
export RECOVER_YAW_RAD="${RECOVER_YAW_RAD:-0.35}"
export WAIT_LOCALIZATION_S="${WAIT_LOCALIZATION_S:-150}"
export RECOVER_WAIT_S="${RECOVER_WAIT_S:-60}"
export MIN_OBS_ACCEPTED="${MIN_OBS_ACCEPTED:-1}"

echo "OUT=$OUT" | tee "$OUT/meta.txt"
echo "DOMAIN=$DOMAIN" | tee -a "$OUT/meta.txt"
echo "PRIOR_PCD=$PRIOR_PCD" | tee -a "$OUT/meta.txt"
echo "TRUTH=($TRUTH_X,$TRUTH_Y,$TRUTH_YAW) WRONG=($WRONG_X,$WRONG_Y,$WRONG_YAW)" | tee -a "$OUT/meta.txt"

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
  launch_small_gicp_relocalization:=true \
  prior_pcd_file:="$PRIOR_PCD" \
  initial_map_to_odom_x:="$WRONG_X" \
  initial_map_to_odom_y:="$WRONG_Y" \
  initial_map_to_odom_yaw:="$WRONG_YAW" \
  gicp_max_correction_translation:=5.0 \
  gicp_max_correction_yaw:=1.5 \
  >"$OUT/launch.log" 2>&1 &
echo $! >"$OUT/launch.pid"
echo "LAUNCH_PID=$(cat "$OUT/launch.pid")"

python3 - <<'PY2' | tee "$OUT/wait_topics.txt"
import os, time, subprocess, sys
need = ["/clock", "/odometry", "/localization", "/registered_scan"]
deadline = time.time() + float(os.environ.get("WAIT_LOCALIZATION_S", "150"))
seen = {t: False for t in need}
gicp = False
while time.time() < deadline:
    try:
        topics = set(subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=8).splitlines())
        nodes = subprocess.check_output(["ros2", "node", "list"], text=True, timeout=8)
    except Exception as exc:
        print("list_fail", exc, flush=True)
        time.sleep(2)
        continue
    for t in need:
        if not seen[t] and t in topics:
            seen[t] = True
            print("seen", t, flush=True)
    if (not gicp) and "small_gicp_relocalization" in nodes:
        gicp = True
        print("seen_node small_gicp_relocalization", flush=True)
    if all(seen.values()) and gicp:
        print("ALL_SEEN", flush=True)
        sys.exit(0)
    time.sleep(2)
print("TIMEOUT", seen, "gicp", gicp, file=sys.stderr)
sys.exit(1)
PY2

{
  echo "==== topic info ===="
  for t in /odometry /localization /registered_scan /relocalization_observation; do
    echo "-- $t"
    ros2 topic info -v "$t" 2>/dev/null | head -35 || true
  done
} | tee "$OUT/graph.txt"

python3 - <<'PY2' | tee "$OUT/reloc_probe.json"
import json, math, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

try:
    from ats_navigation_interfaces.msg import RelocalizationObservation
    HAS_OBS = True
except Exception:
    HAS_OBS = False

truth_x = float(os.environ["TRUTH_X"])
truth_y = float(os.environ["TRUTH_Y"])
truth_yaw = float(os.environ["TRUTH_YAW"])
recover_xy = float(os.environ["RECOVER_XY_M"])
recover_yaw = float(os.environ["RECOVER_YAW_RAD"])
recover_wait = float(os.environ["RECOVER_WAIT_S"])
min_accepted = int(os.environ["MIN_OBS_ACCEPTED"])
reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

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
        super().__init__("gazebo_prior_reloc_probe")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.loc = None
        self.obs_n = 0
        self.obs_accepted = 0
        self.obs_pending = 0
        self.obs_rejected = 0
        self.last_status = None
        self.last_msg = ""
        self.create_subscription(Odometry, "/localization", self.on_loc, qos_profile_sensor_data)
        if HAS_OBS:
            self.create_subscription(
                RelocalizationObservation, "/relocalization_observation", self.on_obs, reliable
            )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def on_loc(self, msg: Odometry):
        self.loc = msg

    def on_obs(self, msg):
        self.obs_n += 1
        self.last_status = int(getattr(msg, "status", -1))
        self.last_msg = str(getattr(msg, "message", ""))
        if bool(getattr(msg, "accepted", False)) or self.last_status == 0:
            self.obs_accepted += 1
        elif self.last_status == 4:
            self.obs_pending += 1
        elif self.last_status == 1:
            self.obs_rejected += 1

    def map_pose(self):
        for frame in ("gimbal_yaw_odom", "base_link", "base_footprint"):
            try:
                tf = self.tf.lookup_transform("map", frame, Time(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                q = tf.transform.rotation
                return t.x, t.y, yaw_of(q), frame
            except Exception:
                continue
        # fallback: localization pose is odom-framed; cannot use alone
        return None

    def publish_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.15
        msg.pose.covariance = cov
        for _ in range(8):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

rclpy.init()
node = Probe()
# warmup
t0 = time.time()
while time.time() - t0 < 12.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)

before = None
for _ in range(50):
    before = node.map_pose()
    if before is not None:
        break
    rclpy.spin_once(node, timeout_sec=0.1)

# Pre-correct error should be large if wrong seed applied
pre_xy = None if before is None else math.hypot(before[0] - truth_x, before[1] - truth_y)
pre_yaw = None if before is None else abs(wrap(before[2] - truth_yaw))

# Seed GICP with TRUE pose guess
node.publish_initialpose(truth_x, truth_y, truth_yaw)

recovered = False
best_xy = 1e9
best_yaw = 1e9
last = None
t0 = time.time()
while time.time() - t0 < recover_wait and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    cur = node.map_pose()
    if cur is None:
        continue
    last = cur
    xy = math.hypot(cur[0] - truth_x, cur[1] - truth_y)
    yw = abs(wrap(cur[2] - truth_yaw))
    best_xy = min(best_xy, xy)
    best_yaw = min(best_yaw, yw)
    if xy <= recover_xy and yw <= recover_yaw and node.obs_accepted >= min_accepted:
        recovered = True
        break

rep = {
    "truth_map_pose": {"x": truth_x, "y": truth_y, "yaw": truth_yaw},
    "pose_before_initialpose": None if before is None else {
        "x": before[0], "y": before[1], "yaw": before[2], "frame": before[3]
    },
    "pre_error_xy_m": pre_xy,
    "pre_error_yaw_rad": pre_yaw,
    "pass_wrong_seed_visible": pre_xy is not None and pre_xy > 0.5,
    "last_map_pose": None if last is None else {
        "x": last[0], "y": last[1], "yaw": last[2], "frame": last[3]
    },
    "obs_n": node.obs_n,
    "obs_accepted": node.obs_accepted,
    "obs_pending": node.obs_pending,
    "obs_rejected": node.obs_rejected,
    "last_obs_status": node.last_status,
    "last_obs_message": node.last_msg,
    "has_obs_msg": HAS_OBS,
    "best_xy_err_m": None if best_xy > 1e8 else best_xy,
    "best_yaw_err_rad": None if best_yaw > 1e8 else best_yaw,
    "min_obs_accepted": min_accepted,
    "pass_obs_accepted": node.obs_accepted >= min_accepted,
    "pass_reloc_recover": recovered,
}
rep["pass"] = bool(rep["pass_wrong_seed_visible"] and rep["pass_obs_accepted"] and recovered)
print(json.dumps(rep, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if rep["pass"] else 4)
PY2
PROBE_RC=$?

python3 - <<PY2 | tee "$OUT/launch_hits.txt"
from pathlib import Path
text = Path("$OUT/launch.log").read_text(errors="ignore")
keys = ("Reject GICP", "accepted", "Loaded global map", "init_pose", "converged", "STATUS_", "small_gicp", "Localization fusion ready")
for i,l in enumerate(text.splitlines(),1):
    if any(k.lower() in l.lower() for k in ("reject gicp", "loaded global map", "localization fusion ready", "awaiting consistent", "accepted relocalization", "not converged", "insufficient inliers")):
        if "voxel coord" in l:
            continue
        print(f"{i}:{l[:240]}")
PY2

if [[ -f "$OUT/launch.pid" ]]; then
  kill -- -"$(cat "$OUT/launch.pid")" 2>/dev/null || true
fi
pkill -9 -f "ats_gazebo_nav|ign gazebo|gz sim|gazebo_gt_|localization_fusion|small_gicp|point_lio|rog_map" 2>/dev/null || true

echo "PROBE_RC=$PROBE_RC"
echo "OUT=$OUT"
exit "$PROBE_RC"
