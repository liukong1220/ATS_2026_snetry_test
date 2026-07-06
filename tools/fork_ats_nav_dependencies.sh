#!/usr/bin/env bash
set -euo pipefail

OWNER="${OWNER:-liukong1220}"
TOPIC="${TOPIC:-ats-nav}"
MAKE_PUBLIC="${MAKE_PUBLIC:-1}"
TOKEN="${GH_TOKEN:-${GITHUB_TOKEN:-${gh_tokens:-}}}"

FORK_REPOSITORIES=(
  "SMBU-PolarBear-Robotics-Team/teleop_gimbal_keyboard:teleop_gimbal_keyboard"
  "LihanChen2004/pcd2pgm:pcd2pgm"
  "berndpfrommer/rosbag2_composable_recorder:rosbag2_composable_recorder"
  "gezp/sdformat_tools:sdformat_tools"
  "LihanChen2004/joint_state_publisher:joint_state_publisher"
  "SMBU-PolarBear-Robotics-Team/rmoss_core:rmoss_core"
  "SMBU-PolarBear-Robotics-Team/rmoss_gazebo:rmoss_gazebo"
  "SMBU-PolarBear-Robotics-Team/rmoss_gz_resources:rmoss_gz_resources"
  "SMBU-PolarBear-Robotics-Team/rmoss_interfaces:rmoss_interfaces"
  "SMBU-PolarBear-Robotics-Team/BehaviorTree.ROS2:BehaviorTree.ROS2"
  "SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator:rmu_gazebo_simulator"
)

PROJECT_REPOSITORIES=(
  "ATS_2026_snetry_test"
  "ats_mujoco_sim"
  "ats_robot_description"
  "ats_sentry_behavior"
  "ats_sentry_nav"
  "carstatemsgs"
  "interfaces"
  "loopback_sim"
  "manda_can_control"
  "sentry_chassis_vel_transform"
  "sp_vision25"
  "standard_robot_pp_ros2"
  "teleop_gimbal_keyboard"
  "pcd2pgm"
  "rosbag2_composable_recorder"
  "sdformat_tools"
  "joint_state_publisher"
  "rmoss_core"
  "rmoss_gazebo"
  "rmoss_gz_resources"
  "rmoss_interfaces"
  "BehaviorTree.ROS2"
  "rmu_gazebo_simulator"
)

usage() {
  cat <<'USAGE'
Usage: GH_TOKEN=... tools/fork_ats_nav_dependencies.sh

Fork upstream tool/dependency repositories into OWNER and tag ATS navigation
repositories with a GitHub topic.

Environment:
  GH_TOKEN/GITHUB_TOKEN/gh_tokens  GitHub token usable by gh CLI.
  OWNER                            Target user or organization, default liukong1220.
  TOPIC                            GitHub topic used as the ATS_NAV group, default ats-nav.
  MAKE_PUBLIC=1|0                  Set target repositories public when possible, default 1.

Notes:
  - GitHub topics are lowercase, so TOPIC=ats-nav represents the ATS_NAV group.
  - For a personal account target, OWNER must match the authenticated GitHub user.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -n "$TOKEN" ]]; then
  export GH_TOKEN="$TOKEN"
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "gh is required. Install GitHub CLI first." >&2
  exit 1
fi

login="$(gh api user --jq .login 2>/dev/null || true)"
if [[ -z "$login" ]]; then
  echo "GitHub authentication is required. Provide GH_TOKEN, GITHUB_TOKEN or gh_tokens." >&2
  exit 1
fi

make_public() {
  local repo="$1"

  if [[ "$MAKE_PUBLIC" == "1" || "$MAKE_PUBLIC" == "true" ]]; then
    printf '{"private":false}' | gh api -X PATCH "repos/${OWNER}/${repo}" --input - >/dev/null
  fi
}

tag_repo() {
  local repo="$1"

  gh repo edit "${OWNER}/${repo}" --add-topic "$TOPIC" >/dev/null
}

fork_repo() {
  local upstream="$1"
  local repo="$2"

  if gh repo view "${OWNER}/${repo}" >/dev/null 2>&1; then
    echo "Exists: ${OWNER}/${repo}"
  else
    echo "Fork: ${upstream} -> ${OWNER}/${repo}"
    if [[ "$OWNER" == "$login" ]]; then
      gh repo fork "$upstream" --clone=false --remote=false >/dev/null
    else
      gh repo fork "$upstream" --org "$OWNER" --clone=false --remote=false >/dev/null
    fi
  fi

  make_public "$repo"
  tag_repo "$repo"
  echo "Ready: ${OWNER}/${repo} topic=${TOPIC}"
}

for spec in "${FORK_REPOSITORIES[@]}"; do
  upstream="${spec%%:*}"
  repo="${spec##*:}"
  fork_repo "$upstream" "$repo"
done

for repo in "${PROJECT_REPOSITORIES[@]}"; do
  if gh repo view "${OWNER}/${repo}" >/dev/null 2>&1; then
    make_public "$repo"
    tag_repo "$repo"
    echo "Grouped: ${OWNER}/${repo} topic=${TOPIC}"
  else
    echo "Skip missing: ${OWNER}/${repo}"
  fi
done
