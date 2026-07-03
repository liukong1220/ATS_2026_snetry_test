#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK_INSTALL_PREFIX="${LIVOX_SDK2_INSTALL_PREFIX:-${PACKAGE_DIR}/Livox-SDK2}"
SDK_REPO_URL="${LIVOX_SDK2_REPO:-https://github.com/Livox-SDK/Livox-SDK2.git}"
SDK_REF="${LIVOX_SDK2_REF:-v1.2.5}"
SDK_CACHE_DIR="${PACKAGE_DIR}/.cache"
SDK_SOURCE_DIR="${SDK_CACHE_DIR}/Livox-SDK2-src"
SDK_BUILD_DIR="${SDK_CACHE_DIR}/Livox-SDK2-build"
BUILD_JOBS="${BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"

rm -rf "${SDK_SOURCE_DIR}" "${SDK_BUILD_DIR}"
mkdir -p "${SDK_CACHE_DIR}"

echo "[Livox SDK2] clone ${SDK_REPO_URL} (${SDK_REF})"
git clone --depth 1 --branch "${SDK_REF}" "${SDK_REPO_URL}" "${SDK_SOURCE_DIR}"

echo "[Livox SDK2] configure -> ${SDK_INSTALL_PREFIX}"
cmake -S "${SDK_SOURCE_DIR}" -B "${SDK_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL_PREFIX}"

echo "[Livox SDK2] build"
cmake --build "${SDK_BUILD_DIR}" --parallel "${BUILD_JOBS}"

echo "[Livox SDK2] install"
cmake --install "${SDK_BUILD_DIR}"

echo
echo "Livox SDK2 is ready at: ${SDK_INSTALL_PREFIX}"
echo "You can now build the workspace directly, or export:"
echo "  export LIVOX_SDK2_ROOT=${SDK_INSTALL_PREFIX}"
