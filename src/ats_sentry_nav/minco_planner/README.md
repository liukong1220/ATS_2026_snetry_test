# minco_planner

This package is the V1 planner-chain workspace for the move from
`RC-ESDF-lite + B-spline + MPPI` toward
`RC-ESDF + A*/JPS + MINCO + independent yaw + footprint safety + local repair`.

Directory layout:

1. `include/minco_planner/planning`, `src/planning`
   Front-end graph search. Current implementation: grid A* over
   `traversability_grid`.
2. `include/minco_planner/trajectory`, `src/trajectory`
   Reference trajectory data structures, temporary time allocation, and
   independent yaw layer. The current optimizer is a placeholder interface for
   the later GCOPTER/MINCO backend.
3. `include/minco_planner/safety`, `src/safety`
   Footprint safety checking and local collision repair.
4. `include/minco_planner/debug`, `src/debug`
   RViz marker generation.
5. `include/minco_planner/nodes`, `src/nodes`
   ROS2 node glue code only.

Reference migration targets:

1. MINCO backend: `~/参考/src/DDR-opt/back_end/include/gcopter/minco.hpp`
2. Footprint SDF collision: `~/参考/src/DDR-opt/utils/plan_env/src/rc_footprint_collision.cpp`
3. MPC / MuJoCo validation: `~/参考/src/nullspace_mpc`,
   `~/参考/src/swerve_drive`, `~/参考/src/MuJoCo-LiDAR`
