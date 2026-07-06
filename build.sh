#!/usr/bin/env bash
set -euo pipefail

# 获取脚本所在目录，并强制切到工作区根目录执行。
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE_DIR}"

echo "[build] starting in ${WORKSPACE_DIR}"

# 默认按 ROS 2 Humble 处理，也允许外部通过 ROS_DISTRO 覆盖。
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

# 清理可能残留的旧 overlay 工作区环境，避免头文件和库串到别的工作区。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
# ROS 官方 setup 脚本里有未定义变量访问，临时关闭 set -u 以保证兼容。
set +u
source "${ROS_SETUP}"
set -u

# 低性能机器默认使用“单包、双核”强度：
# - colcon 同一时间只构建 1 个包
# - 单个 CMake 包内部最多使用 2 个编译任务
# 如需临时调整，可在命令前覆盖：
#   COLCON_WORKERS=2 BUILD_THREADS=4 ./build.sh
COLCON_WORKERS="${COLCON_WORKERS:-1}"
BUILD_THREADS="${BUILD_THREADS:-2}"
CMAKE_CLEAN_CACHE="${CMAKE_CLEAN_CACHE:-0}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${BUILD_THREADS}}"
export MAKEFLAGS="${MAKEFLAGS:--j${BUILD_THREADS}}"

# 这些包通常更重，先单独编译更稳，尤其适合 8G 左右内存的机器。
HEAVY_PACKAGES=(
  livox_ros_driver2
  point_lio
  small_gicp_relocalization
  terrain_analysis
  terrain_analysis_ext
)

has_package() {
  local pkg="$1"
  colcon list --names-only | grep -Fxq "${pkg}"
}

# 启动前先确认关键重包都在当前工作区里，避免脚本跑偏。
echo "[build] checking required packages"
for pkg in "${HEAVY_PACKAGES[@]}"; do
  echo "[build]   - ${pkg}"
  if [[ ! -d "src" ]] || ! has_package "${pkg}"; then
    echo "Required package not found in workspace: ${pkg}" >&2
    exit 1
  fi
done

# 所有构建阶段都复用这一组通用参数。
COMMON_ARGS=(
  --symlink-install
  --cmake-args
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)
if [[ "${CMAKE_CLEAN_CACHE}" == "1" || "${CMAKE_CLEAN_CACHE}" == "true" ]]; then
  COMMON_ARGS=(--cmake-clean-cache "${COMMON_ARGS[@]}")
fi

echo "[build] workspace: ${WORKSPACE_DIR}"
echo "[build] ros distro: ${ROS_DISTRO_NAME}"
echo "[build] colcon workers: ${COLCON_WORKERS}, cmake parallel: ${CMAKE_BUILD_PARALLEL_LEVEL}, makeflags: ${MAKEFLAGS}, clean cache: ${CMAKE_CLEAN_CACHE}"

# 第一阶段先编译重包，降低 OOM 和长链路失败后重头来过的概率。
echo "[build] step 1/2: heavy packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${COLCON_WORKERS}" \
  --packages-select "${HEAVY_PACKAGES[@]}"

# 第二阶段再编译剩余包，这时依赖链已经更稳定，可适当提高并发。
echo "[build] step 2/2: remaining packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${COLCON_WORKERS}" \
  --packages-skip "${HEAVY_PACKAGES[@]}"

echo "[build] done"
echo "[build] next: source install/setup.bash"
