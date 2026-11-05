#!/usr/bin/env python3
"""The full-field fixture must preserve the original physics and frame contract."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from s3_fixture_scene import make_fixture_scene, fixture_site, fixture_position_for_pose


class FullFieldFixture(unittest.TestCase):
    def test_full_fields_keep_all_original_geometry_assets_and_robot_dynamics(self):
        import mujoco

        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        for year in (2025, 2027):
            with self.subTest(year=year), tempfile.TemporaryDirectory() as directory:
                source = simulation / f"models/rmuc_{year}_swerve.xml"
                original = source.read_bytes()
                target = Path(directory) / "field_with_fixture.xml"
                provenance = make_fixture_scene(source, target)
                self.assertEqual(source.read_bytes(), original)
                self.assertTrue((target.parent / "meshes/mid360.obj").is_file())
                before = mujoco.MjModel.from_xml_path(str(source))
                after = mujoco.MjModel.from_xml_path(str(target))
                self.assertEqual(after.ngeom, before.ngeom + 1)
                self.assertEqual(after.nbody, before.nbody + 1)
                self.assertEqual(after.nq, before.nq + 7)
                self.assertEqual(after.nv, before.nv + 6)
                for name in ("geom_type", "geom_size", "geom_pos", "geom_quat",
                             "geom_friction", "geom_contype", "geom_conaffinity",
                             "geom_group", "geom_rgba", "body_pos", "body_quat",
                             "body_mass", "body_inertia", "jnt_type", "jnt_pos", "jnt_axis",
                             "qpos0", "actuator_gear", "sensor_objid", "sensor_type",
                             "mesh_vert", "mesh_face", "hfield_data"):
                    old = getattr(before, name)
                    np.testing.assert_array_equal(getattr(after, name)[:len(old)], old, err_msg=name)
                self.assertEqual(provenance["source_model"], str(source.resolve()))
                self.assertEqual(len(provenance["source_sha256"]), 64)
                for name in ("meanmass", "meaninertia", "meansize", "extent", "center"):
                    np.testing.assert_array_equal(
                        getattr(after.stat, name), getattr(before.stat, name), err_msg=name)
                # The hidden extra body must not change even the independent
                # MuJoCo zero-input response of the complete original robot.
                db, da = mujoco.MjData(before), mujoco.MjData(after)
                # The XML default at (0, 0, .18) intersects the 2025 terrain.
                # Compare the actual production spawn, with the original solver
                # parameters and the same strict numerical assertion below.
                spawn = {2025: [-0.18, 0.06, 0.381], 2027: [4.5, 6.25, 0.465]}[year]
                db.qpos[:7] = [*spawn, 1.0, 0.0, 0.0, 0.0]
                da.qpos[:7] = db.qpos[:7]
                mujoco.mj_forward(before, db)
                mujoco.mj_forward(after, da)
                for _ in range(100):
                    mujoco.mj_step(before, db)
                    mujoco.mj_step(after, da)
                np.testing.assert_allclose(da.qpos[:before.nq], db.qpos, rtol=0, atol=1e-10)

    def test_nonzero_start_and_rotated_map_to_odom_do_not_move_the_fixture_region(self):
        transform = {"translation": {"x": 4.5, "y": 6.25, "z": 0.285},
                     "rotation": {"x": 0.0, "y": 0.0,
                                  "z": 2**-0.5, "w": 2**-0.5}}
        result = fixture_site([4.5, 8.25, 0.585], transform)
        np.testing.assert_allclose(result["odom_position"], [2.0, 0.0, 0.3], atol=1e-12)
        np.testing.assert_allclose(result["odom_half_extent"], [0.3, 0.3, 0.3], atol=1e-12)
        self.assertEqual(result["world_position"], [4.5, 8.25, 0.585])
        transform["rotation"] = dict.fromkeys(("x", "y", "z", "w"), 0.0)
        with self.assertRaises(ValueError):
            fixture_site([4.5, 8.25, 0.585], transform)

    def test_full_field_box_rests_on_actual_ground_instead_of_world_zero(self):
        simulation = Path(__file__).resolve().parents[1] / "src/sim/ats_mujoco_sim"
        position = fixture_position_for_pose(simulation / "models/rmuc_2025_swerve.xml",
                                             [-0.18, 0.06, 0.0])
        np.testing.assert_allclose(position, [1.82, 0.06, 0.200971942595 + 0.3], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
