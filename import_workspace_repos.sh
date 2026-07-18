#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="${MANIFEST:-${ROOT_DIR}/dependencies.repos}"
IMPORT_PATH="${IMPORT_PATH:-${ROOT_DIR}}"

SHALLOW=0
FORCE=0
SYNC_EXISTING=1

usage() {
  cat <<'USAGE'
Usage: ./import_workspace_repos.sh [--shallow] [--force] [--no-sync] [--manifest FILE]

Import repositories listed in dependencies.repos into the workspace root, then
fast-forward existing clean repositories to their manifest versions.

Options:
  --shallow        Clone without full repository history.
  --force          Allow vcstool to replace non-matching existing directories.
  --no-sync        Only import missing repositories; do not update existing ones.
  --manifest FILE  Use a custom .repos manifest.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --shallow)
      SHALLOW=1
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --no-sync)
      SYNC_EXISTING=0
      shift
      ;;
    --manifest)
      MANIFEST="$2"
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

if ! command -v vcs >/dev/null 2>&1; then
  echo "vcstool is not installed. Install python3-vcstool first." >&2
  exit 1
fi

if [[ ! -f "$MANIFEST" ]]; then
  echo "Manifest not found: $MANIFEST" >&2
  exit 1
fi

# Keep deployment output focused without changing the user's global git config.
# vcstool may initialize empty repositories and may checkout pinned commit hashes.
git_config_count="${GIT_CONFIG_COUNT:-0}"
export "GIT_CONFIG_KEY_${git_config_count}=init.defaultBranch"
export "GIT_CONFIG_VALUE_${git_config_count}=main"
git_config_count=$((git_config_count + 1))
export "GIT_CONFIG_KEY_${git_config_count}=advice.detachedHead"
export "GIT_CONFIG_VALUE_${git_config_count}=false"
git_config_count=$((git_config_count + 1))
export GIT_CONFIG_COUNT="$git_config_count"

VCS_ARGS=(--recursive)
if [[ "$FORCE" -eq 1 ]]; then
  VCS_ARGS+=(--force)
else
  VCS_ARGS+=(--skip-existing)
fi
if [[ "$SHALLOW" -eq 1 ]]; then
  VCS_ARGS+=(--shallow)
fi

vcs import "${VCS_ARGS[@]}" --input "$MANIFEST" "$IMPORT_PATH"

if [[ "$SYNC_EXISTING" -eq 0 ]]; then
  exit 0
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required to read the .repos manifest for synchronization." >&2
  exit 1
fi

MANIFEST_REPOS_FILE="$(mktemp)"
trap 'rm -f "$MANIFEST_REPOS_FILE"' EXIT

if ! python3 - "$MANIFEST" >"$MANIFEST_REPOS_FILE" <<'PY'
import os
import sys

import yaml

manifest_path = sys.argv[1]
with open(manifest_path, encoding="utf-8") as manifest_file:
    manifest = yaml.safe_load(manifest_file)

repositories = manifest.get("repositories") if isinstance(manifest, dict) else None
if not isinstance(repositories, dict):
    raise SystemExit("Manifest must contain a 'repositories' mapping.")

for path, specification in repositories.items():
    if not isinstance(path, str) or not isinstance(specification, dict):
        raise SystemExit("Each manifest repository must have a path and mapping.")
    if os.path.isabs(path) or ".." in path.split("/"):
        raise SystemExit(f"Unsafe repository path in manifest: {path!r}")

    repository_type = specification.get("type")
    url = specification.get("url")
    version = specification.get("version")
    if repository_type != "git" or not all(isinstance(value, str) and value for value in (url, version)):
        raise SystemExit(f"Repository {path!r} must define git type, url, and version.")

    sys.stdout.buffer.write(path.encode() + b"\0" + url.encode() + b"\0" + version.encode() + b"\0")
PY
then
  echo "Unable to parse manifest for synchronization: $MANIFEST" >&2
  exit 1
fi

normalize_git_url() {
  local url="$1"
  url="${url%/}"
  printf '%s\n' "${url%.git}"
}

SYNC_UPDATED=0
SYNC_CURRENT=0
SYNC_SKIPPED=0
SYNC_FAILED=0

sync_repository() {
  local repository_path="$1"
  local expected_url="$2"
  local version="$3"
  local repository_dir="${IMPORT_PATH}/${repository_path}"
  local actual_url
  local current_branch
  local fetch_head
  local local_head
  local ls_remote_status
  local version_is_branch=0

  if [[ ! -d "$repository_dir" ]]; then
    echo "[sync:skip] $repository_path: directory was not imported" >&2
    SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    return
  fi

  if ! git -C "$repository_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1 || \
    [[ "$(realpath "$repository_dir")" != "$(realpath "$(git -C "$repository_dir" rev-parse --show-toplevel)")" ]]; then
    echo "[sync:skip] $repository_path: not a Git repository at the manifest path" >&2
    SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    return
  fi

  if [[ -n "$(git -C "$repository_dir" status --porcelain=v1 --untracked-files=all)" ]]; then
    echo "[sync:skip] $repository_path: working tree is not clean" >&2
    SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    return
  fi

  if ! actual_url="$(git -C "$repository_dir" remote get-url origin 2>/dev/null)"; then
    echo "[sync:skip] $repository_path: remote 'origin' is missing" >&2
    SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    return
  fi
  if [[ "$(normalize_git_url "$actual_url")" != "$(normalize_git_url "$expected_url")" ]]; then
    echo "[sync:skip] $repository_path: origin URL differs from the manifest" >&2
    SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    return
  fi

  if git -C "$repository_dir" ls-remote --exit-code --heads origin "refs/heads/$version" >/dev/null 2>&1; then
    version_is_branch=1
  else
    ls_remote_status=$?
    if [[ "$ls_remote_status" -ne 2 ]]; then
      echo "[sync:failed] $repository_path: could not inspect origin/$version" >&2
      SYNC_FAILED=$((SYNC_FAILED + 1))
      return
    fi
  fi

  if [[ "$version_is_branch" -eq 1 ]]; then
    current_branch="$(git -C "$repository_dir" branch --show-current)"
    if [[ "$current_branch" != "$version" ]]; then
      echo "[sync:skip] $repository_path: checked out '$current_branch', manifest requires '$version'" >&2
      SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
      return
    fi

    if ! git -C "$repository_dir" fetch --quiet origin "$version"; then
      echo "[sync:failed] $repository_path: could not fetch origin/$version" >&2
      SYNC_FAILED=$((SYNC_FAILED + 1))
      return
    fi
    fetch_head="$(git -C "$repository_dir" rev-parse --verify FETCH_HEAD^{commit})"
    local_head="$(git -C "$repository_dir" rev-parse --verify HEAD^{commit})"

    if [[ "$local_head" == "$fetch_head" ]]; then
      echo "[sync:current] $repository_path"
      SYNC_CURRENT=$((SYNC_CURRENT + 1))
    elif git -C "$repository_dir" merge-base --is-ancestor "$local_head" "$fetch_head"; then
      if git -C "$repository_dir" pull --ff-only origin "$version"; then
        echo "[sync:updated] $repository_path"
        SYNC_UPDATED=$((SYNC_UPDATED + 1))
      else
        echo "[sync:failed] $repository_path: fast-forward pull failed" >&2
        SYNC_FAILED=$((SYNC_FAILED + 1))
      fi
    elif git -C "$repository_dir" merge-base --is-ancestor "$fetch_head" "$local_head"; then
      echo "[sync:skip] $repository_path: local branch is ahead of origin/$version" >&2
      SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    else
      echo "[sync:skip] $repository_path: local branch diverges from origin/$version" >&2
      SYNC_SKIPPED=$((SYNC_SKIPPED + 1))
    fi
    return
  fi

  if ! git -C "$repository_dir" fetch --quiet origin "$version"; then
    echo "[sync:failed] $repository_path: could not fetch pinned version '$version'" >&2
    SYNC_FAILED=$((SYNC_FAILED + 1))
    return
  fi
  fetch_head="$(git -C "$repository_dir" rev-parse --verify FETCH_HEAD^{commit})"
  local_head="$(git -C "$repository_dir" rev-parse --verify HEAD^{commit})"
  if [[ "$local_head" == "$fetch_head" ]]; then
    echo "[sync:current] $repository_path"
    SYNC_CURRENT=$((SYNC_CURRENT + 1))
  elif git -C "$repository_dir" checkout --detach "$fetch_head"; then
    echo "[sync:updated] $repository_path (pinned version)"
    SYNC_UPDATED=$((SYNC_UPDATED + 1))
  else
    echo "[sync:failed] $repository_path: could not check out pinned version '$version'" >&2
    SYNC_FAILED=$((SYNC_FAILED + 1))
  fi
}

while IFS= read -r -d '' repository_path && \
  IFS= read -r -d '' repository_url && \
  IFS= read -r -d '' repository_version; do
  sync_repository "$repository_path" "$repository_url" "$repository_version"
done <"$MANIFEST_REPOS_FILE"

echo "Synchronization summary: updated=$SYNC_UPDATED current=$SYNC_CURRENT skipped=$SYNC_SKIPPED failed=$SYNC_FAILED"
if [[ "$SYNC_SKIPPED" -ne 0 || "$SYNC_FAILED" -ne 0 ]]; then
  exit 1
fi
