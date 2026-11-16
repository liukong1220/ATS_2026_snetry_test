#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="${MANIFEST:-${ROOT_DIR}/dependencies.repos}"
IMPORT_PATH="${IMPORT_PATH:-${ROOT_DIR}}"
VCS_WORKERS="${VCS_WORKERS:-1}"
NETWORK_ATTEMPTS="${NETWORK_ATTEMPTS:-3}"

SHALLOW=0
FORCE=0
SYNC_EXISTING=1

usage() {
  cat <<'USAGE'
Usage: ./import_workspace_repos.sh [--shallow] [--force] [--no-sync] [--manifest FILE]

Import repositories listed in dependencies.repos into the workspace root, then
fast-forward existing clean repositories to their manifest versions.
GitHub HTTP(S)/git URLs are converted to SSH before any network operation.
Existing remote URLs are migrated to SSH without changing working files.

Options:
  --shallow        Clone without full repository history.
  --force          Allow vcstool to replace non-matching existing directories.
  --no-sync        Only import missing repositories; do not fetch/checkout existing ones.
  --manifest FILE  Use a custom .repos manifest.

VCS_WORKERS controls import concurrency (default: 1).
NETWORK_ATTEMPTS limits each network operation (default: 3).
Git uses SSH only; Git LFS may still use its server's HTTPS API.
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
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "--manifest requires a file path." >&2
        exit 2
      fi
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

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 and PyYAML are required to validate the .repos manifest." >&2
  exit 1
fi

if [[ ! "$VCS_WORKERS" =~ ^[1-9][0-9]*$ || ! "$NETWORK_ATTEMPTS" =~ ^[1-9][0-9]*$ ]]; then
  echo "VCS_WORKERS and NETWORK_ATTEMPTS must be positive integers." >&2
  exit 2
fi

# Keep deployment output focused without changing the user's global git config.
# vcstool may initialize empty repositories and may checkout pinned commit hashes.
git_config_count="${GIT_CONFIG_COUNT:-0}"
append_git_config() {
  export "GIT_CONFIG_KEY_${git_config_count}=$1"
  export "GIT_CONFIG_VALUE_${git_config_count}=$2"
  git_config_count=$((git_config_count + 1))
}
append_git_config init.defaultBranch main
append_git_config advice.detachedHead false
# Also cover HTTPS URLs declared by recursively imported GitHub submodules.
for prefix in https://github.com/ http://github.com/ git://github.com/; do
  append_git_config url.git@github.com:.insteadOf "$prefix"
done
append_git_config protocol.allow never
append_git_config protocol.ssh.allow always
export GIT_CONFIG_COUNT="$git_config_count"

IMPORT_WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$IMPORT_WORK_DIR"' EXIT
SSH_HELPER="${ROOT_DIR}/scripts/workspace_git_ssh.py"
python3 "$SSH_HELPER" prepare "$MANIFEST" "$IMPORT_PATH" "$IMPORT_WORK_DIR" "$FORCE"
MANIFEST_REPOS_FILE="${IMPORT_WORK_DIR}/repositories.bin"

VCS_ARGS=(--recursive --workers "$VCS_WORKERS" --retry "$((NETWORK_ATTEMPTS - 1))")
if [[ "$FORCE" -eq 1 ]]; then
  VCS_ARGS+=(--force)
else
  VCS_ARGS+=(--skip-existing)
fi
if [[ "$SHALLOW" -eq 1 ]]; then
  VCS_ARGS+=(--shallow)
fi

# Existing repositories are handled below after their safety checks. vcstool's
# --skip-existing still fetches matching repositories, even with --no-sync.
if [[ -s "${IMPORT_WORK_DIR}/import.repos" ]]; then
  vcs import "${VCS_ARGS[@]}" --input "${IMPORT_WORK_DIR}/import.repos" "$IMPORT_PATH"
fi
python3 "$SSH_HELPER" migrate "$MANIFEST" "$IMPORT_PATH"

if [[ "$SYNC_EXISTING" -eq 0 ]]; then
  exit 0
fi

normalize_git_url() {
  local url="$1"
  url="${url%/}"
  printf '%s\n' "${url%.git}"
}

git_network() {
  local repository_dir="$1"
  shift
  local attempt
  local status
  for ((attempt = 1; attempt <= NETWORK_ATTEMPTS; attempt++)); do
    if git -C "$repository_dir" "$@"; then
      return 0
    else
      status=$?
    fi
    # ls-remote returns 2 for a missing ref, rather than a connection failure.
    if [[ "$1" == "ls-remote" && "$status" -eq 2 ]]; then
      return "$status"
    fi
    if [[ "$attempt" -lt "$NETWORK_ATTEMPTS" ]]; then
      echo "[network:retry] $repository_dir: $1 failed (attempt $attempt/$NETWORK_ATTEMPTS)" >&2
    fi
  done
  return "$status"
}

SYNC_UPDATED=0
SYNC_CURRENT=0
SYNC_SKIPPED=0
SYNC_FAILED=0

sync_submodules() {
  local repository_dir="$1"
  if [[ -f "$repository_dir/.gitmodules" ]] && \
    ! git_network "$repository_dir" submodule update --init --recursive; then
    echo "[sync:failed] $repository_dir: could not synchronize submodules" >&2
    SYNC_FAILED=$((SYNC_FAILED + 1))
    return 1
  fi
}

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

  if git_network "$repository_dir" ls-remote --exit-code --heads origin "refs/heads/$version" >/dev/null; then
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

    if ! git_network "$repository_dir" fetch --quiet origin "$version"; then
      echo "[sync:failed] $repository_path: could not fetch origin/$version" >&2
      SYNC_FAILED=$((SYNC_FAILED + 1))
      return
    fi
    fetch_head="$(git -C "$repository_dir" rev-parse --verify FETCH_HEAD^{commit})"
    local_head="$(git -C "$repository_dir" rev-parse --verify HEAD^{commit})"

    if [[ "$local_head" == "$fetch_head" ]]; then
      sync_submodules "$repository_dir" || return 0
      echo "[sync:current] $repository_path"
      SYNC_CURRENT=$((SYNC_CURRENT + 1))
    elif git -C "$repository_dir" merge-base --is-ancestor "$local_head" "$fetch_head"; then
      if git -C "$repository_dir" merge --ff-only "$fetch_head"; then
        sync_submodules "$repository_dir" || return 0
        echo "[sync:updated] $repository_path"
        SYNC_UPDATED=$((SYNC_UPDATED + 1))
      else
        echo "[sync:failed] $repository_path: fast-forward merge failed" >&2
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

  if ! git_network "$repository_dir" fetch --quiet origin "$version"; then
    echo "[sync:failed] $repository_path: could not fetch pinned version '$version'" >&2
    SYNC_FAILED=$((SYNC_FAILED + 1))
    return
  fi
  fetch_head="$(git -C "$repository_dir" rev-parse --verify FETCH_HEAD^{commit})"
  local_head="$(git -C "$repository_dir" rev-parse --verify HEAD^{commit})"
  if [[ "$local_head" == "$fetch_head" ]]; then
    sync_submodules "$repository_dir" || return 0
    echo "[sync:current] $repository_path"
    SYNC_CURRENT=$((SYNC_CURRENT + 1))
  elif git -C "$repository_dir" checkout --detach "$fetch_head"; then
    sync_submodules "$repository_dir" || return 0
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

# A fast-forward may introduce new .gitmodules entries and initialized remotes.
python3 "$SSH_HELPER" migrate "$MANIFEST" "$IMPORT_PATH"
echo "Synchronization summary: updated=$SYNC_UPDATED current=$SYNC_CURRENT skipped=$SYNC_SKIPPED failed=$SYNC_FAILED"
if [[ "$SYNC_SKIPPED" -ne 0 || "$SYNC_FAILED" -ne 0 ]]; then
  exit 1
fi
