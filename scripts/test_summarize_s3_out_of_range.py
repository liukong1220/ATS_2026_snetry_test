import unittest

from summarize_s3_out_of_range import summarize


class RangeEvidenceTest(unittest.TestCase):
    def test_missing_log_is_not_zero(self):
        result = summarize('no integration received\n')
        self.assertEqual(result['status'], 'unverified_missing_counter')
        self.assertIsNone(result['latest_logged_counter'])

    def test_counter_is_cumulative_not_sum_of_throttled_reports(self):
        result = summarize(
            'odom layer: column_non_obstacle=3 out_of_range=0 ray_cleared=0\n'
            'S3 global odom layer out_of_range=7 frame_count=2 sample_odom=(21,-16,0.2)\n'
            'odom layer: column_non_obstacle=10 out_of_range=7 ray_cleared=1\n')
        self.assertEqual(result['sum_segment_last_counts_lower_bound'], 7)
        self.assertEqual(result['sampled_positions_odom'], [[21.0, -16.0, 0.2]])

    def test_restart_or_mixed_log_is_explicit_not_silently_monotonic(self):
        result = summarize('S3 global odom layer out_of_range=12\n'
                           'S3 global odom layer out_of_range=1\n'
                           'S3 global odom layer out_of_range=5\n')
        self.assertEqual(result['counter_segments'], [12, 5])
        self.assertEqual(result['counter_decreases'], 1)


if __name__ == '__main__':
    unittest.main()
