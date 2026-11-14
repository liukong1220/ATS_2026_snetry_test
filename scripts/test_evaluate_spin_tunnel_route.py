import math
import json
from pathlib import Path
import tempfile
import unittest

from evaluate_spin_tunnel_route import body_intersects, load_samples, motion_spin_summary


class SpinEvidence(unittest.TestCase):
    def test_parked_peak_does_not_count_as_translating_spin(self):
        rows = [{"t": i*.1, "vx": 0., "vy": 0., "wz": 3.0} for i in range(30)]
        result = motion_spin_summary(rows)
        self.assertEqual(result["moving_samples"], 0)
        self.assertEqual(result["near_target_duration_s"], 0.)
        self.assertIsNone(result["moving_abs_wz_radps"])

    def test_gaps_and_braking_split_the_sustained_interval(self):
        rows = [{"t": t, "vx": 0., "vy": .4, "wz": 3.0}
                for t in [0., .1, .2, 1., 1.1, 1.2]]
        rows[-1]["wz"] = 1.0
        result = motion_spin_summary(rows)
        self.assertAlmostEqual(result["moving_duration_s"], .4)
        self.assertAlmostEqual(result["near_target_duration_s"], .3)
        self.assertAlmostEqual(result["longest_near_target_interval_s"], .2)
        self.assertEqual(result["near_target_intervals_s"], [[0., .2], [1., 1.1]])
        self.assertEqual(result["moving_abs_wz_radps"]["min"], 1.)

    def test_centre_exit_is_not_whole_body_exit(self):
        polygon = [[0., -.5], [2., -.5], [2., .5], [0., .5]]
        row = {"x": 2.1, "y": 0., "yaw": 0.}
        self.assertTrue(body_intersects(row, polygon, .581, .571))
        row["x"] = 2.5
        self.assertFalse(body_intersects(row, polygon, .581, .571))
        row.update(x=2.35, yaw=math.pi/4)
        self.assertTrue(body_intersects(row, polygon, .581, .571))

    def test_repeated_observations_do_not_break_a_sustained_spin(self):
        rows = [{"t": t, "vx": .4, "vy": 0., "wz": 3.0, "leg": "goal_1"}
                for t in [0., .1, .1, .1, .2, .2]]
        result = motion_spin_summary(rows)
        self.assertEqual(result["moving_samples"], 3)
        self.assertAlmostEqual(result["longest_near_target_interval_s"], .2)

    def test_route_leg_boundaries_do_not_join_spin_intervals(self):
        rows = [{"t": t, "vx": .4, "vy": 0., "wz": 3.0, "leg": leg}
                for t, leg in [(0., "goal_1"), (.1, "goal_1"),
                               (.2, "goal_2"), (.3, "goal_2")]]
        result = motion_spin_summary(rows)
        self.assertAlmostEqual(result["moving_duration_s"], .2)
        self.assertAlmostEqual(result["longest_near_target_interval_s"], .1)

    def test_brief_non_open_segment_breaks_open_space_spin(self):
        rows = [{"t": i*.1, "vx": .4, "vy": 0., "wz": 3.0, "open": i != 2}
                for i in range(5)]
        result = motion_spin_summary(rows, eligible=lambda r: r['open'])
        self.assertAlmostEqual(result['moving_duration_s'], .2)
        self.assertAlmostEqual(result['longest_near_target_interval_s'], .1)

    def test_odom_recorder_requires_bounded_map_truth_pairing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            leg = root / 'goal_1'
            leg.mkdir()
            (leg/'summary.json').write_text(json.dumps({'grid_frame': 'odom'}))
            (leg/'samples.jsonl').write_text(json.dumps({
                'actual': {'stamp_identity': '1.000000000', 'x': 0., 'y': 0., 'yaw': 0.},
                'map_from_odom': {'x': 0., 'y': 0., 'yaw': 0.}})+'\n')
            with self.assertRaisesRegex(ValueError, 'odom-grid'):
                load_samples(root)
            truth = root/'truth.csv'
            truth.write_text('sim_s,x,y,yaw\n1.01,3.37,9.03,0.2\n')
            row = load_samples(root, truth)[0]
            self.assertEqual((row['x'], row['y'], row['yaw']), (3.37, 9.03, .2))
            truth.write_text('sim_s,x,y,yaw\n1.2,3.37,9.03,0.2\n')
            with self.assertRaisesRegex(ValueError, 'pairing bound'):
                load_samples(root, truth)


if __name__ == "__main__":
    unittest.main()
