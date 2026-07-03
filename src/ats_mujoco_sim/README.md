# ats_mujoco_sim

MuJoCo-based chassis simulation migrated from the reference swerve chassis
project and adapted for the ATS workspace.

Main entry:

```bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py use_viewer:=true
```

The launch file generates a temporary random map, builds a matching MuJoCo
scene, then starts the simulation node.
Generated maps are written under `/tmp/ats_mujoco_sim_maps` by default; override
`output_root:=...` if you want to keep a specific scene.

Important topics:

1. `/localization` publishes simulated odometry.
2. `/local_pointcloud` publishes optional lidar point cloud data when
   `enable_lidar:=true`.
3. `/perception/tof/points_merged` publishes optional side ToF point clouds.
4. `/simulation/PoseSub` accepts pose-reset commands.

Runtime Python dependencies are expected in the environment:

1. `mujoco`
2. `numpy`
3. `opencv-python` / `cv2`

The `mujoco_lidar` Python package from `~/参考/src/MuJoCo-LiDAR` is vendored into
this ROS2 package so lidar scan generation can be used without a separate clone.
