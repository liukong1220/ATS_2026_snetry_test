#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_PREFIX="${REMOTE_PREFIX:-https://github.com/liukong1220}"
BRANCH="${BRANCH:-develop}"
MODE="${MODE:-subtree}"
WORK_DIR="${WORK_DIR:-${ROOT_DIR}/.split-repos}"
PUSH=0
GIT_USER_NAME=""
GIT_USER_EMAIL=""

REPOSITORIES=(
  "src/sim/ats_mujoco_sim:ats_mujoco_sim"
  "src/ats_robot_description:ats_robot_description"
  "src/ats_sentry_behavior:ats_sentry_behavior"
  "src/ats_sentry_nav:ats_sentry_nav"
  "src/interfaces:interfaces"
  "src/interfaces/carstatemsgs:carstatemsgs"
  "src/sim/loopback_sim:loopback_sim"
  "src/interfaces/manda_can_control:manda_can_control"
  "src/ats_sentry_nav/sentry_chassis_vel_transform:sentry_chassis_vel_transform"
  "src/standard_robot_pp_ros2:standard_robot_pp_ros2"
  "src/sp_vision25:sp_vision25"
)

usage() {
  cat <<'USAGE'
Usage: tools/export_workspace_repos.sh [--push] [--mode subtree|snapshot]

Export local workspace packages into standalone repositories.

src/ats_sentry_bringup is intentionally kept in the root workspace repository
and is not exported as a split repository.

Defaults:
  MODE=subtree
  BRANCH=develop
  REMOTE_PREFIX=https://github.com/liukong1220
  WORK_DIR=.split-repos

Modes:
  subtree   Preserve the history that touched each path with git subtree split.
  snapshot  Create one fresh commit from the current tracked files only.

Notes:
  - GitHub repositories must already exist before using --push.
  - Without --push, the script only creates local split branches or local repos.
  - Uncommitted changes are not included in subtree mode.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --push)
      PUSH=1
      shift
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$MODE" != "subtree" && "$MODE" != "snapshot" ]]; then
  echo "Unsupported mode: $MODE" >&2
  usage >&2
  exit 2
fi

cd "$ROOT_DIR"

GIT_USER_NAME="$(git config --get user.name || true)"
GIT_USER_EMAIL="$(git config --get user.email || true)"

if [[ "$MODE" == "subtree" && "$(git status --porcelain --untracked-files=no)" != "" ]]; then
  echo "Tracked working tree changes exist. Commit or stash them before subtree export." >&2
  exit 1
fi

export_subtree() {
  local path="$1"
  local repo="$2"
  local split_branch="split/${repo}"
  local remote="${REMOTE_PREFIX}/${repo}.git"

  if ! git ls-files --error-unmatch "$path" >/dev/null 2>&1; then
    echo "Skip $path: no tracked files"
    return
  fi

  git branch -D "$split_branch" >/dev/null 2>&1 || true
  git subtree split --prefix="$path" -b "$split_branch"

  if [[ "$PUSH" -eq 1 ]]; then
    git push -u "$remote" "$split_branch:${BRANCH}"
  else
    echo "Created local branch $split_branch for $remote -> $BRANCH"
  fi
}

export_snapshot() {
  local path="$1"
  local repo="$2"
  local dest="${WORK_DIR}/${repo}"
  local remote="${REMOTE_PREFIX}/${repo}.git"

  if ! git ls-files --error-unmatch "$path" >/dev/null 2>&1; then
    echo "Skip $path: no tracked files"
    return
  fi

  rm -rf "$dest"
  mkdir -p "$dest"

  while IFS= read -r -d '' file; do
    local rel="${file#${path}/}"
    mkdir -p "${dest}/$(dirname "$rel")"
    cp -a "$file" "${dest}/${rel}"
  done < <(git ls-files -z "$path")

  (
    cd "$dest"
    git init -q
    if [[ -n "$GIT_USER_NAME" ]]; then
      git config user.name "$GIT_USER_NAME"
    fi
    if [[ -n "$GIT_USER_EMAIL" ]]; then
      git config user.email "$GIT_USER_EMAIL"
    fi
    git checkout -q -b "$BRANCH"
    git add .
    git commit -q -m "Initial split from ATS 2026 workspace"
    git remote add origin "$remote"
    if [[ "$PUSH" -eq 1 ]]; then
      git push -u origin "$BRANCH"
    fi
  )

  if [[ "$PUSH" -eq 0 ]]; then
    echo "Created local snapshot repo $dest for $remote -> $BRANCH"
  fi
}

for spec in "${REPOSITORIES[@]}"; do
  path="${spec%%:*}"
  repo="${spec##*:}"
  case "$MODE" in
    subtree)
      export_subtree "$path" "$repo"
      ;;
    snapshot)
      export_snapshot "$path" "$repo"
      ;;
  esac
done
