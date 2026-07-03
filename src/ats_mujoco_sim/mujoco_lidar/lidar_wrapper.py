from typing import Any

import mujoco
import numpy as np


class MjLidarWrapper:
    """
    MuJoCo LiDAR wrapper supporting CPU, Taichi, and JAX backends.

    Args:
        mj_model: MuJoCo model object
        site_name: Name of the LiDAR site in the model
        backend: 'cpu', 'taichi', or 'jax' (default: 'taichi')
        cutoff_dist: Maximum ray distance in meters (default: 100.0)
        args: Backend-specific arguments (see docs/en/API.md)
    """

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        site_name: str,
        backend: str = "taichi",
        cutoff_dist: float = 100.0,
        args: dict[str, Any] = None,
    ):
        if args is None:
            args = {}
        if backend == "gpu":
            backend = "taichi"
        self.backend = backend
        self.mj_model = mj_model
        self.cutoff_dist = cutoff_dist
        self.args = args

        if backend == "taichi":
            self._init_taichi_backend()
        elif backend == "jax":
            self._init_jax_backend()
        elif backend == "cpu":
            self._init_cpu_backend()
        else:
            raise ValueError(
                f"Unsupported backend: {backend}, choose from 'cpu', 'taichi', or 'jax'"
            )

        self.site_name = site_name
        self._sensor_pose = np.eye(4, dtype=np.float32)
        self._local_rays: np.ndarray | None = None
        self._distances: np.ndarray | None = None
        self._cached_taichi_theta = None
        self._cached_taichi_phi = None
        self._cached_taichi_local_dirs = None
        self._cached_taichi_key: tuple[int, int, tuple[int, ...], tuple[int, ...]] | None = None

    def _init_taichi_backend(self) -> None:
        """Initialize Taichi backend"""
        try:
            # Lazy import: only import when Taichi backend is selected
            import taichi as ti
            from taichi.lang import impl

            from mujoco_lidar.core_ti.mjlidar_ti import MjLidarTi

            # Initialize Taichi if not already done
            if impl.get_runtime().prog is None:
                arch = self.args.get("ti_arch", ti.gpu)
                ti.init(arch=arch, **self.args.get("ti_init_args", {}))

            # Create Taichi backend instance
            geomgroup = self.args.get("geomgroup", None)
            bodyexclude = self.args.get("bodyexclude", -1)
            max_candidates = self.args.get("max_candidates", 64)
            self._backend_instance = MjLidarTi(
                self.mj_model,
                cutoff_dist=self.cutoff_dist,
                geomgroup=geomgroup,
                bodyexclude=bodyexclude,
                max_candidates=max_candidates,
            )

        except ImportError as e:
            raise ImportError(
                f"Failed to import Taichi backend dependencies. "
                f'Please install taichi: uv add "mujoco-lidar[taichi]"\n'
                f"Error: {e}"
            ) from e

    def _init_jax_backend(self) -> None:
        """Initialize JAX backend"""
        try:
            from mujoco_lidar.core_jax.mjlidar_jax import MjLidarJax

            geomgroup = self.args.get("geomgroup", None)
            bodyexclude = self.args.get("bodyexclude", -1)

            # Pass mj_model directly. MjLidarJax will extract what it needs.
            self._backend_instance = MjLidarJax(
                self.mj_model,
                geom_ids=self.args.get("geom_ids"),
                geomgroup=geomgroup,
                bodyexclude=bodyexclude,
            )

        except ImportError as e:
            raise ImportError(f"Failed to import JAX backend dependencies.\nError: {e}") from e

    def _init_cpu_backend(self) -> None:
        """Initialize CPU backend"""
        try:
            from mujoco_lidar.core_cpu.mjlidar_cpu import MjLidarCPU

            geomgroup = self.args.get("geomgroup", None)
            bodyexclude = self.args.get("bodyexclude", -1)
            self._backend_instance = MjLidarCPU(
                self.mj_model,
                cutoff_dist=self.cutoff_dist,
                geomgroup=geomgroup,
                bodyexclude=bodyexclude,
            )

        except ImportError as e:
            raise ImportError(f"Failed to import CPU backend dependencies.\nError: {e}") from e

    @property
    def sensor_position(self) -> np.ndarray:
        return self._sensor_pose[:3, 3].copy()

    @property
    def sensor_rotation(self) -> np.ndarray:
        return self._sensor_pose[:3, :3].copy()

    def update_sensor_pose(self, mj_data: mujoco.MjData, site_name: str) -> None:
        # For CPU/Taichi/JAX backend, mj_data is mujoco.MjData
        if self.backend in ["cpu", "taichi", "jax"]:
            self._sensor_pose[:3, :3] = mj_data.site(site_name).xmat.reshape(3, 3)
            self._sensor_pose[:3, 3] = mj_data.site(site_name).xpos

    def trace_rays(
        self,
        mj_data: mujoco.MjData,
        ray_theta: np.ndarray,
        ray_phi: np.ndarray,
        site_name: str | None = None,
    ) -> np.ndarray:
        """
        Trace rays.
        For JAX backend, mj_data can be mujoco.MjData.
        """
        target_site = self.site_name if site_name is None else site_name

        if self.backend == "jax":
            # Update sensor pose for consistency
            self.update_sensor_pose(mj_data, target_site)

            # Use JITed trace_rays from backend instance
            # This handles ray generation, transformation and rendering in one JIT call
            self._distances, self._local_rays = self._backend_instance.trace_rays(
                mj_data.geom_xpos,
                mj_data.geom_xmat,
                mj_data.site(target_site).xpos,
                mj_data.site(target_site).xmat.reshape(3, 3),
                ray_theta,
                ray_phi,
            )

            return self._distances

        elif self.backend == "taichi":
            # Taichi Backend
            self.update_sensor_pose(mj_data, target_site)
            self._backend_instance.update(mj_data)

            theta_ti, phi_ti, local_dirs_ti = self._as_taichi_rays(ray_theta, ray_phi)
            if local_dirs_ti is not None:
                self._backend_instance.trace_rays_from_dirs(
                    self._sensor_pose,
                    local_dirs_ti,
                )
            else:
                self._backend_instance.trace_rays(self._sensor_pose, theta_ti, phi_ti)
            return self._backend_instance.get_distances()

        else:
            # CPU Backend
            self.update_sensor_pose(mj_data, target_site)
            self._backend_instance.update(mj_data)
            self._backend_instance.trace_rays(self._sensor_pose, ray_theta, ray_phi)
            return self._backend_instance.get_distances()

    def get_hit_points(self) -> np.ndarray:
        if self.backend == "jax":
            if self._distances is None or self._local_rays is None:
                return np.zeros((0, 3), dtype=np.float32)
            return np.asarray(self._distances[:, np.newaxis] * self._local_rays)
        return self._backend_instance.get_hit_points()

    def get_distances(self) -> np.ndarray:
        if self.backend == "jax":
            if self._distances is None:
                return np.zeros(0, dtype=np.float32)
            return np.asarray(self._distances)
        return self._backend_instance.get_distances()

    def trace_points(
        self,
        mj_data: mujoco.MjData,
        ray_theta: np.ndarray,
        ray_phi: np.ndarray,
        min_range: float = 0.0,
        site_name: str | None = None,
    ) -> np.ndarray:
        """Trace rays and return valid local-frame hit points directly."""
        target_site = self.site_name if site_name is None else site_name

        if self.backend == "taichi":
            self.update_sensor_pose(mj_data, target_site)
            self._backend_instance.update(mj_data)
            theta_ti, phi_ti, local_dirs_ti = self._as_taichi_rays(ray_theta, ray_phi)
            if local_dirs_ti is not None:
                return self._backend_instance.trace_valid_points_from_dirs(
                    self._sensor_pose,
                    local_dirs_ti,
                    float(min_range),
                )
            return self._backend_instance.trace_valid_points(
                self._sensor_pose, theta_ti, phi_ti, float(min_range)
            )

        distances = self.trace_rays(mj_data, ray_theta, ray_phi, target_site)
        points = self.get_hit_points()
        valid = np.asarray(distances) >= float(min_range)
        return np.ascontiguousarray(
            np.asarray(points, dtype=np.float32)[valid],
            dtype=np.float32,
        )

    def _as_taichi_rays(self, ray_theta, ray_phi):
        import taichi as ti

        if not isinstance(ray_theta, np.ndarray) and not isinstance(ray_phi, np.ndarray):
            return ray_theta, ray_phi, None
        if not isinstance(ray_theta, np.ndarray) or not isinstance(ray_phi, np.ndarray):
            raise TypeError("ray_theta and ray_phi must both be numpy arrays or Taichi ndarrays")
        if ray_theta.shape != ray_phi.shape:
            raise ValueError("ray_theta and ray_phi must have the same shape")

        cacheable = (
            ray_theta.dtype == np.float32
            and ray_phi.dtype == np.float32
            and ray_theta.flags.c_contiguous
            and ray_phi.flags.c_contiguous
            and not ray_theta.flags.writeable
            and not ray_phi.flags.writeable
        )
        theta_np = np.ascontiguousarray(ray_theta, dtype=np.float32)
        phi_np = np.ascontiguousarray(ray_phi, dtype=np.float32)
        if not cacheable:
            theta_ti = ti.ndarray(dtype=ti.f32, shape=theta_np.shape[0])
            theta_ti.from_numpy(theta_np)
            phi_ti = ti.ndarray(dtype=ti.f32, shape=phi_np.shape[0])
            phi_ti.from_numpy(phi_np)
            return theta_ti, phi_ti, None

        cache_key = (
            int(theta_np.__array_interface__["data"][0]),
            int(phi_np.__array_interface__["data"][0]),
            tuple(theta_np.shape),
            tuple(phi_np.shape),
        )
        if cache_key != self._cached_taichi_key:
            self._cached_taichi_theta = ti.ndarray(dtype=ti.f32, shape=theta_np.shape[0])
            self._cached_taichi_theta.from_numpy(theta_np)
            self._cached_taichi_phi = ti.ndarray(dtype=ti.f32, shape=phi_np.shape[0])
            self._cached_taichi_phi.from_numpy(phi_np)
            local_dirs = np.column_stack((
                np.cos(phi_np) * np.cos(theta_np),
                np.cos(phi_np) * np.sin(theta_np),
                np.sin(phi_np),
            )).astype(np.float32, copy=False)
            self._cached_taichi_local_dirs = ti.ndarray(
                dtype=ti.f32,
                shape=local_dirs.shape,
            )
            self._cached_taichi_local_dirs.from_numpy(local_dirs)
            self._cached_taichi_key = cache_key
        return (
            self._cached_taichi_theta,
            self._cached_taichi_phi,
            self._cached_taichi_local_dirs,
        )
