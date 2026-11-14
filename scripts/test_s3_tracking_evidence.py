import unittest

from s3_tracking_evidence import profile_geometry, verify_runtime_geometry


class TrackingGeometry(unittest.TestCase):
    def test_2027_inherits_simulation_footprint_instead_of_real_vehicle_margin(self):
        geometry, sources = profile_geometry({'launch': 'rmuc_2027_mujoco.launch.py'})
        self.assertEqual(geometry, {'footprint_length': .581, 'footprint_width': .571,
                                    'footprint_safety_margin': .02})
        self.assertEqual(len(sources), 2)
        verify_runtime_geometry(geometry, geometry)
        with self.assertRaisesRegex(ValueError, 'footprint_safety_margin'):
            verify_runtime_geometry(geometry, {**geometry, 'footprint_safety_margin': .05})


if __name__ == '__main__':
    unittest.main()
