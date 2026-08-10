#!/usr/bin/env bash
# 名义实时性定位采集器：先取证据，不先调参。
#
# 三个固定条件实验：
#   A  地图链（ROGMap/LiDAR/投影）在不下发动作跟踪的情况下运行
#   B  完整导航链在更稀疏的 LiDAR 输入下运行，用于验证投影成本是否受点云
#      密度影响；它不是固定/回放 reference，也不构成 MPC-only 实验
#   C  完整名义闭环
#
# 每个实验固定地图、种子、起点、目标与参数，禁用 RViz/viewer，使用独立
# ROS_DOMAIN_ID；进程级 CPU/RSS/线程数/上下文切换只用既有 ps 与 /proc 采集，
# 不引入新依赖。本脚本只采集与汇总，绝不修改任何超时、deadline、lease、
# 20 Hz 控制周期、MPC 约束或安全门禁。
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXPERIMENT="${1:-C}"
OUT_DIR="${OUT_DIR:-/tmp/ats_realtime_profile}"
SAMPLE_INTERVAL_SEC="${SAMPLE_INTERVAL_SEC:-1.0}"
# 固定条件：三个实验共用同一地图、起点与目标，便于逐项比较。
export START_X="${START_X:--10.0}"
export START_Y="${START_Y:-1.47}"
export START_Z="${START_Z:-0.42}"
export START_YAW="${START_YAW:-0.0}"
export LIDAR_DOWNSAMPLE="${LIDAR_DOWNSAMPLE:-2}"
export USE_RVIZ=false
export SOLVER_MODE="${SOLVER_MODE:-ilqr}"
export P2_FAULT_CASE=none

case "${EXPERIMENT}" in
  A) DEFAULT_DOMAIN=201 ;;
  B) DEFAULT_DOMAIN=202 ;;
  C) DEFAULT_DOMAIN=203 ;;
  *)
    echo "usage: $0 {A|B|C}" >&2
    exit 2
    ;;
esac
# 每个实验必须使用独立 domain。ats_sentry_bringup 的 env-hook 会在 ROS_DOMAIN_ID
# 未设置时把它置为 90，所以调用者只要 source 过工作区，`${ROS_DOMAIN_ID:-...}`
# 就永远拿不到本脚本的默认值，三个实验会挤在同一个 domain 上互相污染。这里改用
# 独立变量 PROFILE_DOMAIN 显式覆盖，而不是继承任何已有值。
export ROS_DOMAIN_ID="${PROFILE_DOMAIN:-${DEFAULT_DOMAIN}}"

mkdir -p "${OUT_DIR}"
PROC_SAMPLES="${OUT_DIR}/process_${EXPERIMENT}_${ROS_DOMAIN_ID}.tsv"
RUN_LOG="${OUT_DIR}/run_${EXPERIMENT}_${ROS_DOMAIN_ID}.log"
SUMMARY="${OUT_DIR}/summary_${EXPERIMENT}_${ROS_DOMAIN_ID}.txt"
# MPC 分阶段耗时来自 telemetry ring 的既有导出服务，不新增控制线程内日志。
export QP_TELEMETRY_OUTPUT="${OUT_DIR}/control_telemetry_${EXPERIMENT}_${ROS_DOMAIN_ID}.json"
export QP_TELEMETRY_MANIFEST="${OUT_DIR}/control_telemetry_${EXPERIMENT}_${ROS_DOMAIN_ID}.manifest.json"
# 导出路径非空时窗口必须落在 [1,128]；0 会被回归脚本显式拒绝，因此这里给出
# 一个有界默认值，而不是把导出悄悄关掉。
export QP_TELEMETRY_WINDOW_CYCLES="${QP_TELEMETRY_WINDOW_CYCLES:-64}"
# ROGMap 的 `P2 projection end` 分阶段行是 INFO 级，必须让 info 通过才能采到。
export LOG_LEVEL="${LOG_LEVEL:-info}"

SAMPLER_PID=""
RUN_PID=""
PROFILE_PGID=""

cleanup() {
  [[ -n "${SAMPLER_PID}" ]] && kill "${SAMPLER_PID}" 2>/dev/null || true
  if [[ -n "${RUN_PID}" ]]; then
    kill -INT "-${RUN_PID}" 2>/dev/null || true
    sleep 3
    kill -KILL "-${RUN_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# 只用 ps 与 /proc：线程数取 /proc/<pid>/status Threads，自愿/非自愿上下文切换
# 取同文件的 voluntary_ctxt_switches / nonvoluntary_ctxt_switches。
sample_processes() {
  local target_pgid="$1"
  printf 'epoch_ns\tpid\tpgid\tcomm\tpcpu\trss_kb\tthreads\tvoluntary_ctxt\tnonvoluntary_ctxt\n' \
    >"${PROC_SAMPLES}"
  while :; do
    local epoch_ns
    epoch_ns="$(date +%s%N)"
    while read -r pid pcpu rss comm; do
      [[ -r "/proc/${pid}/status" ]] || continue
      local threads voluntary nonvoluntary
      threads="$(awk '$1 == "Threads:" {print $2; exit}' "/proc/${pid}/status" 2>/dev/null || echo 0)"
      voluntary="$(awk '$1 == "voluntary_ctxt_switches:" {print $2; exit}' \
        "/proc/${pid}/status" 2>/dev/null || echo 0)"
      nonvoluntary="$(awk '$1 == "nonvoluntary_ctxt_switches:" {print $2; exit}' \
        "/proc/${pid}/status" 2>/dev/null || echo 0)"
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "${epoch_ns}" "${pid}" "${target_pgid}" "${comm}" "${pcpu}" "${rss}" \
        "${threads:-0}" "${voluntary:-0}" "${nonvoluntary:-0}" >>"${PROC_SAMPLES}"
    done < <(ps -eo pid=,pgid=,pcpu=,rss=,comm= | awk -v pgid="${target_pgid}" '
      $2 == pgid && $5 ~ /(ats_rog_map|ats_swerve_mpc|minco|mujoco|localization|goal_manager|python3)/ {
        print $1, $3, $4, $5
      }')
    sleep "${SAMPLE_INTERVAL_SEC}"
  done
}

# A/B/C 的差异只在负载条件，不在任何超时或门禁参数。
case "${EXPERIMENT}" in
  A)
    # 地图链单独运行：不发布导航目标，因此不会进入 tracking，MPC 保持零速度。
    # 回归脚本读取这两个变量后跳过目标动作段，并改为断言两级命令保持零速度。
    export ATS_PROFILE_SKIP_ACTION=1
    export ATS_PROFILE_MAP_OBSERVE_SEC="${ATS_PROFILE_MAP_OBSERVE_SEC:-90}"
    ;;
  B)
    # 只降低 LiDAR 采样密度，完整导航/规划/MPC 链仍然运行。因此它只能回答
    # 投影成本是否随点云密度变化，不能被解释为固定 reference 的 MPC-only 实验。
    export LIDAR_DOWNSAMPLE="${PROFILE_B_LIDAR_DOWNSAMPLE:-8}"
    ;;
  C)
    : # 完整名义闭环，保持默认。
    ;;
esac

echo "experiment=${EXPERIMENT} domain=${ROS_DOMAIN_ID} out=${OUT_DIR}"
setsid bash "${WORKSPACE_DIR}/scripts/test_mujoco_minco_mpc_chain.sh" \
  >"${RUN_LOG}" 2>&1 &
RUN_PID="$!"
for _ in $(seq 1 50); do
  PROFILE_PGID="$(ps -o pgid= -p "${RUN_PID}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${PROFILE_PGID}" =~ ^[0-9]+$ ]] && break
  sleep 0.1
done
[[ "${PROFILE_PGID}" =~ ^[0-9]+$ ]] || {
  echo "cannot resolve process group for profile launch pid ${RUN_PID}" >&2
  exit 1
}
sample_processes "${PROFILE_PGID}" &
SAMPLER_PID="$!"
RUN_STATUS=0
wait "${RUN_PID}" || RUN_STATUS="$?"
RUN_PID=""
kill "${SAMPLER_PID}" 2>/dev/null || true
SAMPLER_PID=""

# `P2 projection end` 分阶段行由 ros2 launch 写进 LAUNCH_LOG，回归脚本的 stdout
# 只在失败时 tail 它的尾部。按 stdout 汇总会在通过的运行里得到 0 个样本，在失败
# 的运行里得到被截断的样本，两种都是错的证据，所以这里显式读 launch 日志。
LAUNCH_LOG="/tmp/ats_minco_mpc_test_launch_${ROS_DOMAIN_ID}.log"
cp -f "${LAUNCH_LOG}" "${OUT_DIR}/launch_${EXPERIMENT}_${ROS_DOMAIN_ID}.log" 2>/dev/null || true

python3 "${WORKSPACE_DIR}/scripts/summarize_realtime_profile.py" \
  --run-log "${LAUNCH_LOG}" \
  --process-samples "${PROC_SAMPLES}" \
  --control-telemetry "${QP_TELEMETRY_OUTPUT}" \
  --experiment "${EXPERIMENT}" \
  --domain "${ROS_DOMAIN_ID}" | tee "${SUMMARY}"

echo "run_status=${RUN_STATUS} run_pid=${RUN_PID:-exited} process_group=${PROFILE_PGID} run_log=${RUN_LOG} process_samples=${PROC_SAMPLES} summary=${SUMMARY}"
exit "${RUN_STATUS}"
