#!/usr/bin/env python3
import unittest

from analyze_o0_handoffs import measure


class HandoffMeasurement(unittest.TestCase):
    def fixture(self, drop=False, stop=False, terminal=False):
        records = []

        def add(t, kind, data):
            records.append({'ros_time_ns': round(t * 1e9), 'kind': kind, 'data': data})

        for kind, data in [('emergency_stop', False), ('map_ready', True)]:
            add(0, kind, {'data': data})
        add(0, 'localization_status', {'state': 1})
        add(0.95, 'localization', {'header': {'frame_id': 'odom'},
                                 'pose': {'pose': {'position': {'x': 4.9 if terminal else 0, 'y': 0}}}})
        for t, reference_stamp in [(0.5, 500000000), (1.0, 1000000000)]:
            add(t, 'execution', {'mode': 1, 'manager_incarnation': 1, 'goal_id': 2,
                                'localization_epoch': 3,
                                'reference': {'stamp_ns': reference_stamp, 'frame': 'odom',
                                              'poses': [[0, 0, 0, 0], [1, 5, 0, 0]]}})
        if stop:
            add(1.01, 'emergency_stop', {'data': True})
        for step in range(14, 27):
            t = step * 0.05
            v = 0.1 if drop and t >= 1.0 else 1.0
            add(t, 'horizon', {'poses': [[0, 0, 0, 0], [0, 0.05, 0, 0]]})
            twist = {'linear': {'x': v, 'y': 0}, 'angular': {'z': 0}}
            for kind in ('autonomy_raw', 'selected'):
                add(t, kind, twist)
            add(t, 'motion_control', {'linear_x': v, 'linear_y': 0, 'angular_z': 0})
            add(t, 'truth', {'header': {'stamp': {'sec': int(t), 'nanosec': round(t % 1 * 1e9)}},
                             'twist': {'twist': twist}})
        return records

    def test_nominal_handoff_and_reset_drop_are_distinguished(self):
        good, _ = measure(self.fixture(), 0.05)
        self.assertTrue(good['o0_passed'])
        bad, _ = measure(self.fixture(drop=True), 0.05)
        self.assertFalse(bad['o0_passed'])
        self.assertAlmostEqual(bad['events'][-1]['metrics']['truth']['drop_mps'], 0.9)

    def test_safety_and_terminal_samples_remain_in_report(self):
        for kwargs, classification in [({'stop': True}, 'safety_transition'),
                                       ({'terminal': True}, 'terminal_braking')]:
            result, rows = measure(self.fixture(drop=True, **kwargs), 0.05)
            self.assertEqual(result['handoffs'], 1)
            self.assertFalse(result['o0_passed'])
            self.assertEqual(result['eligible_cruise_handoffs'], 0)
            self.assertIn(classification, result['events'][-1]['classification'])
            self.assertFalse(result['events'][-1]['observed_drop_limit_passed'])
            self.assertEqual(result['all_classifications_drop_failures'], 1)
            self.assertTrue(rows)

    def test_missing_truth_and_untriggered_events_cannot_pass(self):
        records = [r for r in self.fixture() if r['kind'] != 'truth']
        result, _ = measure(records, 0.05)
        self.assertFalse(result['o0_passed'])
        self.assertIn('incomplete_speed_window', result['events'][-1]['classification'])
        result, _ = measure([], 0.05)
        self.assertFalse(result['o0_passed'])
        self.assertEqual(result['o0_status'], 'not_triggered')

    def test_wall_clock_odometry_stop_is_preserved_separately(self):
        records = self.fixture(drop=True)
        for record in records:
            record['received_steady_s'] = record['ros_time_ns'] * 1e-8
        result, _ = measure(records, 0.05, odometry_timeout=1.0)
        event = result['events'][-1]
        self.assertIn('odometry_receive_timeout', event['classification'])
        self.assertFalse(event['observed_drop_limit_passed'])
        self.assertFalse(result['o0_passed'])


if __name__ == '__main__':
    unittest.main()
