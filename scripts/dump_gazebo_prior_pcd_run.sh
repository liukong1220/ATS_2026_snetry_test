#!/usr/bin/env bash
# Launch GT Gazebo (GICP off), dump /registered_scan -> map-frame prior PCD.
set -eo pipefail
WS="${WS:-/home/kong/ATS_2026_snetry_test}"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

DOMAIN="${ROS_DOMAIN_ID:-$((40 + RANDOM % 60))}"
export ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY=1
OUT_PCD="${OUT_PCD:-$WS/src/ats_sentry_bringup/pcd/rmuc_2025_gazebo_fullfield.pcd}"
LOG_DIR="$WS/log/gazebo_prior_dump/$(date +%Y%m%d_%H%M%S)_domain${DOMAIN}"
mkdir -p "$(dirname "$OUT_PCD")" "$LOG_DIR"
DURATION="${DURATION:-18}"

pkill -9 -f 'ats_gazebo_nav|ign gazebo|gz sim|gazebo_gt_|localization_fusion|small_gicp|ros2 launch' 2>/dev/null || true
sleep 2
free -h | head -2 | tee "$LOG_DIR/mem.txt"

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
  initial_map_to_odom_x:=1.17 \
  initial_map_to_odom_y:=-0.44 \
  initial_map_to_odom_yaw:=0.0 \
  >"$LOG_DIR/launch.log" 2>&1 &
echo $! >"$LOG_DIR/launch.pid"

python3 - <<'PY' | tee "$LOG_DIR/wait.txt"
import time, subprocess, sys
need=["/clock","/odometry","/localization","/registered_scan"]
seen={t:False for t in need}
deadline=time.time()+120
while time.time()<deadline:
    try:
        topics=set(subprocess.check_output(["ros2","topic","list"],text=True,timeout=8).splitlines())
    except Exception as e:
        print("fail",e); time.sleep(2); continue
    for t in need:
        if not seen[t] and t in topics:
            seen[t]=True; print("seen",t,flush=True)
    if all(seen.values()):
        print("ALL_SEEN"); sys.exit(0)
    time.sleep(2)
print("TIMEOUT",seen); sys.exit(1)
PY

sleep 5
python3 "$WS/scripts/dump_gazebo_prior_pcd.py" \
  --output "$OUT_PCD" \
  --cloud-topic /registered_scan \
  --map-frame map \
  --duration "$DURATION" \
  --voxel 0.08 \
  | tee "$LOG_DIR/dump.txt"
RC=${PIPESTATUS[0]}

if [[ -f "$LOG_DIR/launch.pid" ]]; then
  kill -- -"$(cat "$LOG_DIR/launch.pid")" 2>/dev/null || true
fi
pkill -9 -f 'ats_gazebo_nav|ign gazebo|gz sim|gazebo_gt_|localization_fusion' 2>/dev/null || true
ls -lh "$OUT_PCD" 2>/dev/null | tee "$LOG_DIR/pcd_ls.txt" || true
echo "OUT_PCD=$OUT_PCD RC=$RC LOG=$LOG_DIR"
exit "$RC"
