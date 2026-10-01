#!/usr/bin/env python3
"""静态检查导航运行日志的中文可读性、节流入口和回归键。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, source: Path) -> None:
    if needle not in text:
        raise AssertionError(f"{source}: 缺少 {needle!r}")


def main() -> int:
    planner = ROOT / "src/ats_sentry_nav/minco_planner/src/nodes/minco_planner_node.cpp"
    gicp = ROOT / (
        "src/ats_sentry_nav/small_gicp_relocalization/src/"
        "small_gicp_relocalization.cpp"
    )
    mpc = ROOT / "src/ats_sentry_nav/ats_swerve_mpc/src/ats_swerve_mpc_node.cpp"
    gazebo = ROOT / (
        "src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/"
        "chassis_cmd_adapter.py"
    )

    planner_text = planner.read_text()
    gicp_text = gicp.read_text()
    mpc_text = mpc.read_text()
    gazebo_text = gazebo.read_text()

    for text, source in (
        (planner_text, planner),
        (gicp_text, gicp),
        (mpc_text, mpc),
        (gazebo_text, gazebo),
    ):
        require(text, "【", source)

    require(planner_text, '"log_throttle_ms"', planner)
    require(planner_text, "RCLCPP_WARN_THROTTLE", planner)
    require(planner_text, "planned generation=", planner)
    require(planner_text, "jps failed:", planner)
    require(gicp_text, '"log_throttle_ms"', gicp)
    require(gicp_text, "RCLCPP_INFO_THROTTLE", gicp)
    require(gicp_text, "GICP结果", gicp)
    require(mpc_text, "TRACE execution_command", mpc)
    require(mpc_text, "【执行授权", mpc)
    require(gazebo_text, "【Gazebo底盘接口】", gazebo)
    require(gazebo_text, "throttle_duration_sec=2.0", gazebo)

    print("PASS: 中文运行日志与节流契约")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
