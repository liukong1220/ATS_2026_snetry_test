from pathlib import Path

import numpy as np

from h_stationary_fixture import make_stationary_fixture


def test_world_weld_preserves_geometry_and_solver_inputs_while_holding_the_ramp_pose(tmp_path):
    import mujoco
    source = Path(__file__).resolve().parents[1] / 'src/sim/ats_mujoco_sim/models/rmuc_2027_swerve.xml'
    original = source.read_bytes()
    target = tmp_path / 'stationary.xml'
    start = [9.3, 2.76, .508, -np.pi/2]
    provenance = make_stationary_fixture(source, target, start, pitch=-.23672697)
    assert source.read_bytes() == original
    assert 'not parking or navigation' in provenance['scope']
    before = mujoco.MjModel.from_xml_path(str(source))
    after = mujoco.MjModel.from_xml_path(str(target))
    for name in ['geom_size', 'geom_type', 'geom_friction', 'geom_solref', 'geom_solimp',
                 'body_mass', 'body_inertia', 'actuator_ctrlrange', 'hfield_data', 'mesh_vert']:
        np.testing.assert_array_equal(getattr(before, name), getattr(after, name))
    np.testing.assert_array_equal(before.opt.gravity, after.opt.gravity)
    for name in ['meanmass', 'meaninertia', 'meansize', 'extent', 'center']:
        np.testing.assert_allclose(getattr(before.stat, name), getattr(after.stat, name))
    assert after.neq == before.neq + 1
    from ats_mujoco_sim.model_patch import patch_lidar_mount
    from ats_mujoco_sim.lidar_mount import chassis_mount
    # Match the actual launch mount before testing physical contact/holding.
    # The source XML retains an obsolete gimbal mount for reproduction only.
    after = mujoco.MjModel.from_xml_string(patch_lidar_mount(target.read_text(), chassis_mount()))
    data = mujoco.MjData(after)
    initial = data.qpos[:7].copy()
    np.testing.assert_allclose(initial[:3], start[:3])
    for _ in range(2000):
        mujoco.mj_step(after, data)
    # Solver-visible support gives stable pose AND consistent zero velocity and
    # acceleration, unlike clearing velocity after recording the IMU sample.
    assert np.linalg.norm(data.qpos[:3] - initial[:3]) < .005
    assert abs(np.dot(data.qpos[3:7], initial[3:7])) > .99999
    assert np.linalg.norm(data.qvel[:6]) < .001
    assert np.linalg.norm(data.qacc[:6]) < .01
