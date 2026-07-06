#!/usr/bin/env bash
set -euo pipefail

OWNER="${OWNER:-liukong1220}"
PRIVATE="${PRIVATE:-0}"
MAKE_PUBLIC="${MAKE_PUBLIC:-1}"
TOKEN="${GH_TOKEN:-${GITHUB_TOKEN:-}}"

REPOSITORIES=(
  ats_mujoco_sim
  ats_robot_description
  ats_sentry_behavior
  ats_sentry_nav
  carstatemsgs
  interfaces
  loopback_sim
  manda_can_control
  sentry_chassis_vel_transform
  sp_vision25
  standard_robot_pp_ros2
)

usage() {
  cat <<'USAGE'
Usage: GH_TOKEN=... tools/create_github_repos.sh

Create the split package repositories under OWNER, defaulting to liukong1220.

Environment:
  GH_TOKEN/GITHUB_TOKEN  GitHub token with repo creation permission.
  OWNER                  GitHub user or organization, default liukong1220.
  PRIVATE=1|0            Create private repositories, default 0.
  MAKE_PUBLIC=1|0        Set existing repositories public when possible, default 1.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "$TOKEN" ]]; then
  echo "GH_TOKEN or GITHUB_TOKEN is required." >&2
  usage >&2
  exit 1
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required." >&2
  exit 1
fi

api() {
  curl -fsS \
    -H "Accept: application/vnd.github+json" \
    -H "Authorization: Bearer ${TOKEN}" \
    -H "X-GitHub-Api-Version: 2022-11-28" \
    "$@"
}

login="$(api https://api.github.com/user | python3 -c 'import json,sys; print(json.load(sys.stdin)["login"])')"

if [[ "$PRIVATE" == "1" || "$PRIVATE" == "true" ]]; then
  private_json=true
else
  private_json=false
fi

for repo in "${REPOSITORIES[@]}"; do
  if api -o /dev/null "https://api.github.com/repos/${OWNER}/${repo}" 2>/dev/null; then
    echo "Exists: ${OWNER}/${repo}"
    if [[ "$MAKE_PUBLIC" == "1" || "$MAKE_PUBLIC" == "true" ]]; then
      api -X PATCH "https://api.github.com/repos/${OWNER}/${repo}" \
        -d '{"private":false}' >/dev/null
      echo "Public: ${OWNER}/${repo}"
    fi
    continue
  fi

  payload="$(python3 - <<PY
import json
print(json.dumps({
    "name": "$repo",
    "private": "$private_json" == "true",
    "auto_init": False,
}))
PY
)"

  if [[ "$OWNER" == "$login" ]]; then
    api -X POST https://api.github.com/user/repos -d "$payload" >/dev/null
  else
    api -X POST "https://api.github.com/orgs/${OWNER}/repos" -d "$payload" >/dev/null
  fi
  echo "Created: ${OWNER}/${repo}"
done
