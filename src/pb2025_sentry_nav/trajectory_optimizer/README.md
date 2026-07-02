# trajectory_optimizer

This package is the current Nav2-compatible B-spline smoothing and RC-ESDF
transition layer.

Directory layout:

1. `include/trajectory_optimizer/bspline`, `src/bspline`
   B-spline path smoothing, continuous optimization, curvature / slope /
   obstacle speed-profile generation.
2. `include/trajectory_optimizer/esdf`, `src/esdf`
   ESDF providers and the RC-ESDF-lite traversability backend.
   `rc_traversability_esdf_provider` uses positive distance in free space and
   negative distance inside obstacles.
3. `include/trajectory_optimizer/nav2`, `src/nav2`
   Nav2 smoother plugin wrapper.
4. `include/trajectory_optimizer/control`, `src/control`
   Speed-governor node consuming trajectory profiles and controller commands.
5. `include/trajectory_optimizer/nodes`, `src/nodes`
   Standalone trajectory optimizer / debug node.

Keep new implementation work in the matching layer directory. Avoid adding new
planning, controller, or ESDF logic directly into a node wrapper.
