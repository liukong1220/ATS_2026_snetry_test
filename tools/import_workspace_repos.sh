#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${MANIFEST:-${ROOT_DIR}/dependencies.repos}"
IMPORT_PATH="${IMPORT_PATH:-${ROOT_DIR}}"

SHALLOW=0
FORCE=0

usage() {
  cat <<'USAGE'
Usage: tools/import_workspace_repos.sh [--shallow] [--force] [--manifest FILE]

Import repositories listed in dependencies.repos into the workspace root.

Options:
  --shallow        Clone without full repository history.
  --force          Allow vcstool to replace non-matching existing directories.
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
