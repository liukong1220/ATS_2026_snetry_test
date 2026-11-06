"""Exercise the runner's tunnel profile without starting ROS or MuJoCo."""

from pathlib import Path
import math
import subprocess
import unittest


RUNNER = Path(__file__).with_name("test_mujoco_minco_mpc_chain.sh")


class TunnelStartYawTest(unittest.TestCase):
    def run_profile(self, start, yaw=None):
        text = RUNNER.read_text(encoding="utf-8")
        defaults = text.split('START_YAW="${START_YAW:-0.0}"', 1)[1].split(
            'GOAL_X="', 1
        )[0]
        profile = text.split("  tunnel_2027)\n", 1)[1].split(
            "    ;;", 1
        )[0]
        commands = [
            "set -eu",
            "require_static_map_current() { :; }",
            'SCENE=rmuc_2027; TUNNEL=tunnel_ramp_bridge',
            f'WORKSPACE_DIR="{RUNNER.parent.parent}"',
            'START_Z_USER_SET=0; GOAL_TIMEOUT=300',
            f'TUNNEL_START="{start}"',
            f'START_YAW="{yaw}"' if yaw is not None else "unset START_YAW",
            'START_YAW="${START_YAW:-0.0}"' + defaults,
            profile,
            'printf "%s" "$START_YAW"',
        ]
        return float(subprocess.check_output(
            ["bash", "-c", "\n".join(commands)], text=True
        ))

    def test_base_defaults_to_zero(self):
        self.assertEqual(self.run_profile("3.37 9.03"), 0.0)

    def test_base_preserves_explicit_yaw(self):
        self.assertEqual(self.run_profile("3.37 9.03", "0.7"), 0.7)

    def test_without_base_uses_tunnel_entry(self):
        self.assertAlmostEqual(self.run_profile(""), -1.5707963267948966)


class TunnelGoalYawTest(unittest.TestCase):
    def run_goals(self, start="3.37 9.03", tunnels="tunnel_ramp_bridge,tunnel_B",
                  test_profile="tunnel_2027"):
        text = RUNNER.read_text(encoding="utf-8")
        functions = text.split("goal_quaternion() {", 1)[1].split(
            'if [[ -v GOAL_YAW_W ]]', 1
        )[0]
        profile = text.split('case "${TEST_PROFILE}" in\n', 1)[1].split(
            "\nesac", 1
        )[0]
        # Run the actual per-action orientation setup, without any ROS process.
        setup = text.split("run_navigation_goal() {\n", 1)[1].split(
            '  local prefix=', 1
        )[0]
        orientations = text.split(
            '# 未指定逐目标朝向的 profile', 1
        )[1].split('if [[ "${SCENE}"', 1)[0].split('\n', 1)[1]
        commands = [
            "set -eu",
            "require_static_map_current() { :; }",
            f'WORKSPACE_DIR="{RUNNER.parent.parent}"',
            f'SCENE=rmuc_2027; TEST_PROFILE={test_profile}',
            f'TUNNEL="{tunnels}"; TUNNEL_START="{start}"',
            'TUNNEL_START_YAW=0; START_Z_USER_SET=0; GOAL_TIMEOUT=300',
            'TUNNEL_VIA="4 10"; TUNNEL_FINAL="9.8 11.7"',
            'GOAL_YAW=0.7; GOAL_YAWS=()',
            "goal_quaternion() {" + functions,
            'GOAL_QUATERNION="$(goal_quaternion "$GOAL_YAW")"',
            'read -r GOAL_YAW_Z GOAL_YAW_W <<<"$GOAL_QUATERNION"',
            'case "${TEST_PROFILE}" in\n' + profile + "\nesac",
            orientations,
            "setup_goal() {\n" + setup + '\n  printf "%s %s %s\\n" '
            '"$GOAL_YAW" "$GOAL_YAW_Z" "$GOAL_YAW_W"\n}',
            'for index in "${!GOAL_NAMES[@]}"; do setup_goal "$index"; done',
        ]
        result = subprocess.check_output(["bash", "-c", "\n".join(commands)], text=True)
        return [tuple(map(float, row.split())) for row in result.splitlines()]

    def assert_orientation(self, result, expected):
        self.assertEqual(len(result), len(expected))
        for (yaw, z, w), wanted in zip(result, expected):
            self.assertAlmostEqual(yaw, wanted)
            self.assertAlmostEqual(z, math.sin(wanted / 2))
            self.assertAlmostEqual(w, math.cos(wanted / 2))

    def test_each_tunnel_action_uses_its_own_yaw(self):
        self.assert_orientation(self.run_goals(), [
            0.7, -math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.7,
        ])

    def test_direct_tunnel_exit_recomputes_quaternion(self):
        self.assert_orientation(self.run_goals("", "tunnel_ramp_bridge"), [-math.pi / 2])

    def test_six_tunnels_keep_mirrored_orientations(self):
        self.assert_orientation(self.run_goals(tunnels=",".join([
            "tunnel_ramp_bridge", "tunnel_ramp_bridge_mirror",
            "tunnel_south_plate", "tunnel_south_plate_mirror",
            "tunnel_B", "tunnel_B_mirror",
        ])), [0.7, -math.pi / 2, -math.pi / 2, math.pi / 2, math.pi / 2,
              0.0, 0.0, math.pi, math.pi, 0.0, 0.0, math.pi, math.pi, 0.7])

    def test_non_tunnel_profile_retains_default_yaw(self):
        self.assert_orientation(self.run_goals(test_profile="rectangle"), [0.7] * 5)


if __name__ == "__main__":
    unittest.main()
