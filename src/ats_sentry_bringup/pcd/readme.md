# Prior PCD (RMUC 2025)

权威仿真先验（与 `map/rmuc_2025.yaml` / `.pgm` 同框）：

- `rmuc_2025.pcd` — 规范名（`world:=rmuc_2025` 默认解析到此文件）
- `rmuc_2025_gazebo_fullfield.pcd` — STL 全场采样生成物（同内容）
- `rmuc_2025_gazebo_prior.pcd` — 兼容别名（同内容）

生成：从 Gazebo `rmuc_2025.stl` 采样，变换 `p_map = p_stl + (10.92, -1.44, 0.20)`，
体素 `0.03 m`，高度带 `[-0.2, 3.0]`。

`rmul_2025.pcd` 仅用于 RMUL 场地，与 RMUC 无关。
