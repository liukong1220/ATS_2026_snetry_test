#!/usr/bin/env bash
set -euo pipefail

# 获取脚本所在目录，并强制切到工作区根目录执行。
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WORKSPACE_DIR}"

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

# 读取机器总内存和 CPU 核数，用来自动决定构建并发策略。
MEM_KB="$(awk '/MemTotal/ {print $2}' /proc/meminfo)"
CPU_COUNT="$(nproc)"

# 这些包通常更重，先单独编译更稳，尤其适合 8G 左右内存的机器。
HEAVY_PACKAGES=(
  livox_ros_driver2
  point_lio
  small_gicp_relocalization
  terrain_analysis
  terrain_analysis_ext
)

# 启动前先确认关键重包都在当前工作区里，避免脚本跑偏。
for pkg in "${HEAVY_PACKAGES[@]}"; do
  if [[ ! -d "src" ]] || ! grep -Rqs "<name>${pkg}</name>" src; then
    echo "Required package not found in workspace: ${pkg}" >&2
    exit 1
  fi
done

# 按内存大小自动降并发。
# 7~8G 机器保守串行，16G 以下适度并发，16G 以上再提高普通包速度。
if (( MEM_KB < 8 * 1024 * 1024 )); then
  HEAVY_WORKERS=1
  OTHER_WORKERS=2
  export CMAKE_BUILD_PARALLEL_LEVEL=1
elif (( MEM_KB < 16 * 1024 * 1024 )); then
  HEAVY_WORKERS=4
  OTHER_WORKERS=8
  export CMAKE_BUILD_PARALLEL_LEVEL=2
else
  HEAVY_WORKERS=8
  OTHER_WORKERS="$(( CPU_COUNT > 8 ? 8 : CPU_COUNT ))"
  export CMAKE_BUILD_PARALLEL_LEVEL="$(( CPU_COUNT > 8 ? 8 : CPU_COUNT ))"
fi

# 所有构建阶段都复用这一组通用参数。
COMMON_ARGS=(
  --symlink-install
  --cmake-args
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

echo "[build] workspace: ${WORKSPACE_DIR}"
echo "[build] ros distro: ${ROS_DISTRO_NAME}"
echo "[build] mem: $(( MEM_KB / 1024 / 1024 )) GB, cpu: ${CPU_COUNT}"
echo "[build] heavy workers: ${HEAVY_WORKERS}, other workers: ${OTHER_WORKERS}, cmake parallel: ${CMAKE_BUILD_PARALLEL_LEVEL}"

# 第一阶段先编译重包，降低 OOM 和长链路失败后重头来过的概率。
echo "[build] step 1/2: heavy packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${HEAVY_WORKERS}" \
  --packages-select "${HEAVY_PACKAGES[@]}"

# 第二阶段再编译剩余包，这时依赖链已经更稳定，可适当提高并发。
echo "[build] step 2/2: remaining packages"
colcon build \
  "${COMMON_ARGS[@]}" \
  --parallel-workers "${OTHER_WORKERS}" \
  --packages-skip "${HEAVY_PACKAGES[@]}"

echo "[build] done"
echo "[build] next: source install/setup.bash"
