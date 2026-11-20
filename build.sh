#!/usr/bin/env bash
set -euo pipefail

# 获取脚本所在目录，并切到工作区根目录执行。
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE_DIR}"

echo "[build] starting in ${WORKSPACE_DIR}"

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[build] ERROR: ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

# 清理可能残留的旧 overlay 环境，避免头文件与库串到其他工作区。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

# 按当前机器核心数与可用内存计算安全并行度（每编译任务默认预算 1536 MiB）。
BUILD_MEM_PER_JOB_MB="${BUILD_MEM_PER_JOB_MB:-1536}"
CPU_COUNT="$(nproc)"
MEM_AVAILABLE_MB="$(awk '/^MemAvailable:/ {print int($2 / 1024)}' /proc/meminfo)"
TOTAL_JOBS=$(( MEM_AVAILABLE_MB / BUILD_MEM_PER_JOB_MB ))
(( TOTAL_JOBS > CPU_COUNT )) && TOTAL_JOBS="${CPU_COUNT}"
(( TOTAL_JOBS < 1 )) && TOTAL_JOBS=1

AUTO_WORKERS=$(( TOTAL_JOBS / 4 ))
(( AUTO_WORKERS < 1 )) && AUTO_WORKERS=1
(( AUTO_WORKERS > 4 )) && AUTO_WORKERS=4

COLCON_WORKERS="${COLCON_WORKERS:-${AUTO_WORKERS}}"
BUILD_THREADS="${BUILD_THREADS:-$(( TOTAL_JOBS / COLCON_WORKERS > 0 ? TOTAL_JOBS / COLCON_WORKERS : 1 ))}"
CMAKE_CLEAN_CACHE="${CMAKE_CLEAN_CACHE:-0}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${BUILD_THREADS}}"
export MAKEFLAGS="${MAKEFLAGS:--j${BUILD_THREADS}}"

COMMON_ARGS=(
  --base-paths src
  --symlink-install
  --cmake-args
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_TESTING=OFF
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)
if [[ "${CMAKE_CLEAN_CACHE}" == "1" || "${CMAKE_CLEAN_CACHE}" == "true" ]]; then
  COMMON_ARGS=(--cmake-clean-cache "${COMMON_ARGS[@]}")
fi

echo "[build] workspace: ${WORKSPACE_DIR}"
echo "[build] ros distro: ${ROS_DISTRO_NAME}"
echo "[build] cpu: ${CPU_COUNT}, mem available: ${MEM_AVAILABLE_MB} MiB, budget/job: ${BUILD_MEM_PER_JOB_MB} MiB, total jobs: ${TOTAL_JOBS}"
echo "[build] colcon workers: ${COLCON_WORKERS}, cmake parallel: ${CMAKE_BUILD_PARALLEL_LEVEL}, makeflags: ${MAKEFLAGS}, clean cache: ${CMAKE_CLEAN_CACHE}"

# 若外部传入参数（例如 --packages-select ...），直接透传给 colcon build。
if (( $# > 0 )); then
  colcon build \
    "${COMMON_ARGS[@]}" \
    --parallel-workers "${COLCON_WORKERS}" \
    "$@"
  echo "[build] done"
  echo "[build] next: source install/setup.bash"
  exit 0
fi

# 整仓构建：先编译重型包及其依赖，再编译剩余包。
HEAVY_PACKAGES=(
  livox_ros_driver2
  point_lio
  small_gicp_relocalization
  terrain_analysis
  terrain_analysis_ext
)

has_package() {
  local pkg="$1"
  colcon list --base-paths src --names-only | grep -Fxq "${pkg}"
}

echo "[build] checking required packages"
for pkg in "${HEAVY_PACKAGES[@]}"; do
  echo "[build]   - ${pkg}"
  if [[ ! -d "src" ]] || ! has_package "${pkg}"; then
    echo "[build] ERROR: Required package not found in workspace: ${pkg}" >&2
    exit 1
  fi
done

echo "[build] step 1/2: heavy packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${COLCON_WORKERS}" \
  --packages-up-to "${HEAVY_PACKAGES[@]}"

echo "[build] step 2/2: remaining packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${COLCON_WORKERS}" \
  --packages-skip "${HEAVY_PACKAGES[@]}"

echo "[build] done"
echo "[build] next: source install/setup.bash"
