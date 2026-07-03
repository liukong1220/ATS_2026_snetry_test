"""Generate planner map files and matching swerve MuJoCo scenes."""

from __future__ import annotations

import argparse
import ast
import secrets
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .map_metadata import load_occupancy_image
from .map_metadata import read_map_metadata
from .dynamic_obstacles import DynamicObstacleSceneConfig
from .scene_assets import generate_scene_assets


def package_root() -> Path:
    """Return the source package root."""
    return Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class MapArtifacts:
    map_dir: Path
    model_dir: Path
    map_image: Path
    global_map_image: Path
    map_yaml: Path
    global_map_yaml: Path
    fields_yaml: Path
    manifest_yaml: Path
    ready_file: Path
    terrain_image: Path | None
    scene_xml: Path


@dataclass(frozen=True)
class MapConfig:
    output_root: Path
    map_name: str = ""
    seed: int = 7
    width: int = 420
    height: int = 420
    resolution: float = 0.05
    obstacle_count: int = 60
    min_obstacle_size: int = 10
    max_obstacle_size: int = 34
    border_thickness: int = 5
    corridor_half_width: int = 8
    pixel_threshold: int = 230
    terrain_mode: str = "hfield"
    terrain_height_scale: float = 0.15
    terrain_negative_height: float = 0.03
    terrain_blur_sigma: float = 0.0
    input_map_image: Path | None = None
    input_map_yaml: Path | None = None
    ready_token: str = ""
    dynamic_obstacles_enabled: bool = False
    dynamic_obstacle_count: int = 0
    dynamic_obstacle_shape: str = "cylinder"
    dynamic_obstacle_radius: float = 0.25
    dynamic_obstacle_height: float = 1.0
    dynamic_obstacle_mass: float = 20.0
    dynamic_obstacle_speed: float = 0.4
    dynamic_obstacle_path_length: float = 6.0
    dynamic_obstacle_min_robot_distance: float = 1.8
    dynamic_obstacle_near_robot_radius: float = 4.0
    dynamic_obstacle_map_clearance: float = 0.35
    dynamic_obstacle_replan_rate_hz: float = 1.0


DEFAULT_CONFIG_VALUES: dict[str, Any] = {
    "output_root": (package_root() / "map").as_posix(),
    "map_name": "",
    "seed": 7,
    "width": 420,
    "height": 420,
    "resolution": 0.05,
    "obstacle_count": 60,
    "min_obstacle_size": 10,
    "max_obstacle_size": 34,
    "border_thickness": 5,
    "corridor_half_width": 8,
    "pixel_threshold": 230,
    "terrain_mode": "hfield",
    "terrain_height_scale": 0.15,
    "terrain_negative_height": 0.03,
    "terrain_blur_sigma": 0.0,
    "input_map_image": "",
    "input_map_yaml": "",
    "ready_token": "",
    "dynamic_obstacles_enabled": False,
    "dynamic_obstacle_count": 0,
    "dynamic_obstacle_shape": "cylinder",
    "dynamic_obstacle_radius": 0.25,
    "dynamic_obstacle_height": 1.0,
    "dynamic_obstacle_mass": 20.0,
    "dynamic_obstacle_speed": 0.4,
    "dynamic_obstacle_path_length": 6.0,
    "dynamic_obstacle_min_robot_distance": 1.8,
    "dynamic_obstacle_near_robot_radius": 4.0,
    "dynamic_obstacle_map_clearance": 0.35,
    "dynamic_obstacle_replan_rate_hz": 1.0,
}


def _parse_scalar(value: str) -> Any:
    value = value.strip()
    if value == "":
        return ""

    lower_value = value.lower()
    if lower_value in ("true", "false"):
        return lower_value == "true"
    if lower_value in ("null", "none"):
        return None

    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        pass

    try:
        return int(value)
    except ValueError:
        pass

    try:
        return float(value)
    except ValueError:
        return value


def load_flat_yaml(path: Path) -> dict[str, Any]:
    """Read a simple key/value YAML file without requiring extra packages."""
    values: dict[str, Any] = {}
    path = path.expanduser().resolve()
    with path.open("r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                raise ValueError(f"Invalid config line {path}:{line_number}: {raw_line.rstrip()}")
            key, value = line.split(":", 1)
            key = key.strip().replace("-", "_")
            if not key:
                raise ValueError(f"Empty config key at {path}:{line_number}")
            values[key] = _parse_scalar(value)
    return values


def load_config_defaults(config_path: str | Path) -> dict[str, Any]:
    """Load generator defaults from a flat YAML config file."""
    if not config_path:
        return {}

    config_values = load_flat_yaml(Path(config_path))
    unknown_keys = sorted(set(config_values) - set(DEFAULT_CONFIG_VALUES))
    if unknown_keys:
        raise ValueError(
            "Unknown map config keys: "
            + ", ".join(unknown_keys)
            + ". Supported keys are: "
            + ", ".join(sorted(DEFAULT_CONFIG_VALUES))
        )
    return config_values


def expand_package_path(path_text: str | Path) -> Path:
    """Expand ~ and {package_root} in file paths used by the generator."""
    text = str(path_text).strip()
    text = text.replace("{package_root}", package_root().as_posix())
    return Path(text).expanduser()


def map_seed_dir_name(seed: int) -> str:
    return "random" if seed == -1 else str(seed)


def map_output_dir(output_root: Path, map_name: str, seed: int) -> Path:
    base_dir = Path(output_root).expanduser().resolve()
    seed_dir = map_seed_dir_name(seed)
    if map_name:
        return base_dir / map_name / seed_dir
    return base_dir / seed_dir


def write_yaml_lines(path: Path, lines: list[str]) -> None:
    """Write a small YAML file from preformatted lines."""
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_map_yaml(
    path: Path,
    image_name: str,
    resolution: float,
    origin: tuple[float, float, float],
) -> None:
    """Write ROS occupancy-map metadata."""
    write_yaml_lines(
        path,
        [
            f"image: {image_name}",
            f"resolution: {resolution}",
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
            "occupied_thresh: 0.5",
            "free_thresh: 0.2",
            "negate: 0",
        ],
    )


def write_fields_yaml(
    path: Path,
    base_name: str,
    resolution: float,
    origin: tuple[float, float, float],
    pixel_threshold: int,
    width: int,
    height: int,
) -> None:
    """Write lightweight metadata for generated planner fields."""
    x_lower = float(origin[0])
    y_lower = float(origin[1])
    x_upper = x_lower + float(width) * float(resolution)
    y_upper = y_lower + float(height) * float(resolution)
    write_yaml_lines(
        path,
        [
            f"source_image: {base_name}.png",
            f"planning_image: {base_name}.png",
            f"resolution: {resolution}",
            f"planning_grid_resolution: {resolution}",
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
            f"width: {width}",
            f"height: {height}",
            f"pcd_map_x_lower: {x_lower}",
            f"pcd_map_x_upper: {x_upper}",
            f"pcd_map_y_lower: {y_lower}",
            f"pcd_map_y_upper: {y_upper}",
            f"occupied_pixel_threshold: {pixel_threshold}",
            "occupied_rule: \"pixel < threshold => occupied\"",
        ],
    )


def write_manifest_yaml(
    path: Path,
    config: MapConfig,
    map_dir: Path,
    map_source: str,
    resolved_seed: int | None,
    resolution: float,
    origin: tuple[float, float, float],
    pixel_threshold: int,
    width: int,
    height: int,
    map_image_path: Path,
    global_map_image_path: Path,
    map_yaml_path: Path,
    global_map_yaml_path: Path,
    fields_yaml_path: Path,
    ready_file_path: Path,
    scene_xml_path: Path,
    terrain_image_path: Path | None,
) -> None:
    """Write a manifest so plan_manager can consume a generated map directory."""
    x_lower = float(origin[0])
    y_lower = float(origin[1])
    x_upper = x_lower + float(width) * float(resolution)
    y_upper = y_lower + float(height) * float(resolution)
    write_yaml_lines(
        path,
        [
            f"map_dir: {map_dir.as_posix()}",
            f"map_source: {map_source}",
            f"map_name: {config.map_name}",
            f"seed: {config.seed}",
            f"resolved_seed: {'' if resolved_seed is None else resolved_seed}",
            f"resolution: {resolution}",
            f"width: {width}",
            f"height: {height}",
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
            f"pcd_map_x_lower: {x_lower}",
            f"pcd_map_x_upper: {x_upper}",
            f"pcd_map_y_lower: {y_lower}",
            f"pcd_map_y_upper: {y_upper}",
            f"occupied_pixel_threshold: {pixel_threshold}",
            "map_input_method: 3",
            f"map_image: {map_image_path.as_posix()}",
            f"global_map_image: {global_map_image_path.as_posix()}",
            f"map_yaml: {map_yaml_path.as_posix()}",
            f"global_map_yaml: {global_map_yaml_path.as_posix()}",
            f"map_fields: {fields_yaml_path.as_posix()}",
            f"scene_file: {scene_xml_path.as_posix()}",
            f"terrain_image: {'' if terrain_image_path is None else terrain_image_path.as_posix()}",
            f"ready_file: {ready_file_path.as_posix()}",
            f"ready_token: {config.ready_token}",
            f"input_map_image: {'' if config.input_map_image is None else Path(config.input_map_image).as_posix()}",
            f"input_map_yaml: {'' if config.input_map_yaml is None else Path(config.input_map_yaml).as_posix()}",
            f"dynamic_obstacles_enabled: {str(config.dynamic_obstacles_enabled).lower()}",
            f"dynamic_obstacle_count: {config.dynamic_obstacle_count}",
            f"dynamic_obstacle_shape: {config.dynamic_obstacle_shape}",
            f"dynamic_obstacle_radius: {config.dynamic_obstacle_radius}",
            f"dynamic_obstacle_height: {config.dynamic_obstacle_height}",
            f"dynamic_obstacle_mass: {config.dynamic_obstacle_mass}",
            f"dynamic_obstacle_speed: {config.dynamic_obstacle_speed}",
            f"dynamic_obstacle_path_length: {config.dynamic_obstacle_path_length}",
            f"dynamic_obstacle_min_robot_distance: {config.dynamic_obstacle_min_robot_distance}",
            f"dynamic_obstacle_near_robot_radius: {config.dynamic_obstacle_near_robot_radius}",
            f"dynamic_obstacle_map_clearance: {config.dynamic_obstacle_map_clearance}",
            f"dynamic_obstacle_replan_rate_hz: {config.dynamic_obstacle_replan_rate_hz}",
        ],
    )


def _copy_if_needed(source_path: Path, target_path: Path) -> None:
    source_path = source_path.expanduser().resolve()
    target_path = target_path.expanduser().resolve()
    if source_path == target_path:
        return
    target_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_path, target_path)


def _stage_external_map_assets(
    config: MapConfig,
    map_dir: Path,
) -> tuple[np.ndarray, float, tuple[float, float, float], Path]:
    if config.input_map_image is None or config.input_map_yaml is None:
        raise ValueError(
            "Both input_map_image and input_map_yaml are required for external map mode."
        )

    source_image = Path(config.input_map_image).expanduser().resolve()
    source_yaml = Path(config.input_map_yaml).expanduser().resolve()
    if not source_image.exists():
        raise FileNotFoundError(f"External map image not found: {source_image}")
    if not source_yaml.exists():
        raise FileNotFoundError(f"External map yaml not found: {source_yaml}")

    map_image_path = map_dir / "map.png"
    global_map_image_path = map_dir / "global_map.png"
    map_yaml_path = map_dir / "map.yaml"
    global_map_yaml_path = map_dir / "global_map.yaml"

    _copy_if_needed(source_image, map_image_path)
    _copy_if_needed(source_image, global_map_image_path)

    resolution, origin = read_map_metadata(source_yaml)
    write_map_yaml(map_yaml_path, "map.png", resolution, origin)
    write_map_yaml(global_map_yaml_path, "global_map.png", resolution, origin)

    occupancy = load_occupancy_image(global_map_image_path, config.pixel_threshold)
    gray = np.where(occupancy, 0, 255).astype(np.uint8)
    return gray, resolution, origin, global_map_image_path


def generate_random_occupancy(
    width: int,
    height: int,
    rng: np.random.Generator,
    obstacle_count: int,
    min_obstacle_size: int,
    max_obstacle_size: int,
    border_thickness: int,
    corridor_half_width: int,
) -> np.ndarray:
    """Create a simple random occupancy image for quick simulations."""
    gray = np.full((height, width), 255, dtype=np.uint8)

    gray[:border_thickness, :] = 0
    gray[-border_thickness:, :] = 0
    gray[:, :border_thickness] = 0
    gray[:, -border_thickness:] = 0

    inner_left = border_thickness
    inner_top = border_thickness
    inner_right = max(inner_left + 1, width - border_thickness)
    inner_bottom = max(inner_top + 1, height - border_thickness)

    for _ in range(max(0, obstacle_count)):
        obstacle_width = min(
            int(rng.integers(min_obstacle_size, max_obstacle_size + 1)),
            max(1, inner_right - inner_left - 1),
        )
        obstacle_height = min(
            int(rng.integers(min_obstacle_size, max_obstacle_size + 1)),
            max(1, inner_bottom - inner_top - 1),
        )
        x = int(
            rng.integers(
                inner_left,
                max(inner_left + 1, inner_right - obstacle_width) + 1,
            )
        )
        y = int(
            rng.integers(
                inner_top,
                max(inner_top + 1, inner_bottom - obstacle_height) + 1,
            )
        )

        if rng.random() < 0.65:
            cv2.rectangle(
                gray,
                (x, y),
                (x + obstacle_width, y + obstacle_height),
                color=0,
                thickness=-1,
            )
        else:
            x2 = int(
                np.clip(
                    x + int(rng.integers(-obstacle_width, obstacle_width + 1)),
                    inner_left,
                    width - 1,
                )
            )
            y2 = int(
                np.clip(
                    y + int(rng.integers(-obstacle_height, obstacle_height + 1)),
                    inner_top,
                    height - 1,
                )
            )
            thickness = int(rng.integers(3, 8))
            cv2.line(gray, (x, y), (x2, y2), color=0, thickness=thickness)

    obstacle_mask = cv2.dilate(
        (gray < 128).astype(np.uint8),
        np.ones((3, 3), np.uint8),
        iterations=1,
    ) > 0
    gray[obstacle_mask] = 0

    cx = width // 2
    cy = height // 2
    corridor = max(1, corridor_half_width)
    gray[:, max(0, cx - corridor):min(width, cx + corridor + 1)] = 255
    gray[max(0, cy - corridor):min(height, cy + corridor + 1), :] = 255

    clear_radius = max(corridor + 4, min(width, height) // 24)
    gray[
        max(0, cy - clear_radius):min(height, cy + clear_radius + 1),
        max(0, cx - clear_radius):min(width, cx + clear_radius + 1),
    ] = 255

    gray[:border_thickness, :] = 0
    gray[-border_thickness:, :] = 0
    gray[:, :border_thickness] = 0
    gray[:, -border_thickness:] = 0
    return gray


def generate_map_assets(config: MapConfig) -> MapArtifacts:
    """Generate planner map files and a matching MuJoCo scene."""
    map_dir = map_output_dir(config.output_root, config.map_name, config.seed)
    model_dir = map_dir / "model"
    # 输出目录
    map_dir.mkdir(parents=True, exist_ok=True)
    model_dir.mkdir(parents=True, exist_ok=True)

    map_image_path = map_dir / "map.png"
    global_map_image_path = map_dir / "global_map.png"
    map_yaml_path = map_dir / "map.yaml"
    global_map_yaml_path = map_dir / "global_map.yaml"

    resolved_seed: int | None = None
    if config.input_map_image is not None or config.input_map_yaml is not None:
        # 有外部的 map image 和 yaml 文件
        gray, resolution, origin, source_map_path = _stage_external_map_assets(config, map_dir)
        map_source = "external"
    else:
        # seed=-1 表示每次都重新随机；其它 seed 保持可复现。
        resolved_seed = secrets.randbits(32) if config.seed == -1 else config.seed
        rng = np.random.default_rng(resolved_seed)
        # 生成一张灰度占据栅格图
        gray = generate_random_occupancy(
            width=config.width,
            height=config.height,
            rng=rng,
            obstacle_count=config.obstacle_count,
            min_obstacle_size=config.min_obstacle_size,
            max_obstacle_size=config.max_obstacle_size,
            border_thickness=config.border_thickness,
            corridor_half_width=config.corridor_half_width,
        )
        resolution = config.resolution
        origin = (
            -0.5 * float(config.width) * resolution,
            -0.5 * float(config.height) * resolution,
            0.0,
        )
        cv2.imwrite(map_image_path.as_posix(), gray)
        cv2.imwrite(global_map_image_path.as_posix(), gray)
        # 输出 png 和 yaml 文件
        write_map_yaml(map_yaml_path, "map.png", resolution, origin)
        write_map_yaml(global_map_yaml_path, "global_map.png", resolution, origin)
        source_map_path = global_map_image_path
        map_source = "random"

    fields_yaml_path = map_dir / "global_map_fields.yaml"
    write_fields_yaml(
        fields_yaml_path,
        "global_map",
        resolution,
        origin,
        config.pixel_threshold,
        gray.shape[1],
        gray.shape[0],
    )

    # 生成 mujoco 场景
    scene_artifacts = generate_scene_assets(
        map_dir=map_dir,
        model_dir=model_dir,
        scene_name="swerve",
        pixel_threshold=config.pixel_threshold,
        terrain_mode=config.terrain_mode,
        terrain_height_scale=config.terrain_height_scale,
        terrain_negative_height=config.terrain_negative_height,
        terrain_blur_sigma=config.terrain_blur_sigma,
        dynamic_obstacles=DynamicObstacleSceneConfig(
            enabled=config.dynamic_obstacles_enabled,
            count=config.dynamic_obstacle_count,
            shape=config.dynamic_obstacle_shape,
            radius=config.dynamic_obstacle_radius,
            height=config.dynamic_obstacle_height,
            mass=config.dynamic_obstacle_mass,
            speed=config.dynamic_obstacle_speed,
            path_length=config.dynamic_obstacle_path_length,
            min_robot_distance=config.dynamic_obstacle_min_robot_distance,
            near_robot_radius=config.dynamic_obstacle_near_robot_radius,
            map_clearance=config.dynamic_obstacle_map_clearance,
            replan_rate_hz=config.dynamic_obstacle_replan_rate_hz,
        ),
    )

    occupied_pixels = int(np.count_nonzero(gray < config.pixel_threshold))
    ready_file_path = map_dir / "assets.ready"
    manifest_yaml_path = map_dir / "map_manifest.yaml"
    write_manifest_yaml(
        manifest_yaml_path,
        config,
        map_dir,
        map_source,
        resolved_seed,
        resolution,
        origin,
        config.pixel_threshold,
        gray.shape[1],
        gray.shape[0],
        map_image_path,
        global_map_image_path,
        map_yaml_path,
        global_map_yaml_path,
        fields_yaml_path,
        ready_file_path,
        scene_artifacts.scene_out,
        scene_artifacts.terrain_image,
    )
    write_yaml_lines(
        ready_file_path,
        [
            f"ready_token: {config.ready_token}",
            f"map_dir: {map_dir.as_posix()}",
            f"map_image: {global_map_image_path.as_posix()}",
            f"map_fields: {fields_yaml_path.as_posix()}",
            f"map_manifest: {manifest_yaml_path.as_posix()}",
            f"scene_file: {scene_artifacts.scene_out.as_posix()}",
        ],
    )
    print(
        f"Prepared {map_source} map: {source_map_path} "
        f"({occupied_pixels} occupied pixels), "
        f"resolved_seed={'' if resolved_seed is None else resolved_seed}, "
        f"scene={scene_artifacts.scene_out}"
    )

    return MapArtifacts(
        map_dir=map_dir,
        model_dir=model_dir,
        map_image=map_image_path,
        global_map_image=global_map_image_path,
        map_yaml=map_yaml_path,
        global_map_yaml=global_map_yaml_path,
        fields_yaml=fields_yaml_path,
        manifest_yaml=manifest_yaml_path,
        ready_file=ready_file_path,
        terrain_image=scene_artifacts.terrain_image,
        scene_xml=scene_artifacts.scene_out,
    )


def _build_parser(defaults: dict[str, Any] | None = None) -> argparse.ArgumentParser:
    values = dict(DEFAULT_CONFIG_VALUES)
    if defaults:
        values.update(defaults)

    parser = argparse.ArgumentParser(
        description="Generate swerve planner map assets and a MuJoCo scene."
    )
    parser.add_argument("--config", default="")
    parser.add_argument("--output-root", default=values["output_root"])
    parser.add_argument("--map-name", default=values["map_name"])
    parser.add_argument("--seed", type=int, default=values["seed"], help="-1 means random every run")
    parser.add_argument("--width", type=int, default=values["width"])
    parser.add_argument("--height", type=int, default=values["height"])
    parser.add_argument("--resolution", type=float, default=values["resolution"])
    parser.add_argument("--obstacle-count", type=int, default=values["obstacle_count"])
    parser.add_argument("--min-obstacle-size", type=int, default=values["min_obstacle_size"])
    parser.add_argument("--max-obstacle-size", type=int, default=values["max_obstacle_size"])
    parser.add_argument("--border-thickness", type=int, default=values["border_thickness"])
    parser.add_argument("--corridor-half-width", type=int, default=values["corridor_half_width"])
    parser.add_argument("--pixel-threshold", type=int, default=values["pixel_threshold"])
    parser.add_argument("--terrain-mode", choices=["hfield", "dynamic"], default=values["terrain_mode"])
    parser.add_argument("--terrain-height-scale", type=float, default=values["terrain_height_scale"])
    parser.add_argument("--terrain-negative-height", type=float, default=values["terrain_negative_height"])
    parser.add_argument("--terrain-blur-sigma", type=float, default=values["terrain_blur_sigma"])
    parser.add_argument("--input-map-image", default=values["input_map_image"] or "")
    parser.add_argument("--input-map-yaml", default=values["input_map_yaml"] or "")
    parser.add_argument("--ready-token", default=values["ready_token"] or "")
    parser.add_argument(
        "--dynamic-obstacles-enabled",
        action=argparse.BooleanOptionalAction,
        default=values["dynamic_obstacles_enabled"],
    )
    parser.add_argument("--dynamic-obstacle-count", type=int, default=values["dynamic_obstacle_count"])
    parser.add_argument("--dynamic-obstacle-shape", default=values["dynamic_obstacle_shape"])
    parser.add_argument("--dynamic-obstacle-radius", type=float, default=values["dynamic_obstacle_radius"])
    parser.add_argument("--dynamic-obstacle-height", type=float, default=values["dynamic_obstacle_height"])
    parser.add_argument("--dynamic-obstacle-mass", type=float, default=values["dynamic_obstacle_mass"])
    parser.add_argument("--dynamic-obstacle-speed", type=float, default=values["dynamic_obstacle_speed"])
    parser.add_argument("--dynamic-obstacle-path-length", type=float, default=values["dynamic_obstacle_path_length"])
    parser.add_argument("--dynamic-obstacle-min-robot-distance", type=float, default=values["dynamic_obstacle_min_robot_distance"])
    parser.add_argument("--dynamic-obstacle-near-robot-radius", type=float, default=values["dynamic_obstacle_near_robot_radius"])
    parser.add_argument("--dynamic-obstacle-map-clearance", type=float, default=values["dynamic_obstacle_map_clearance"])
    parser.add_argument("--dynamic-obstacle-replan-rate-hz", type=float, default=values["dynamic_obstacle_replan_rate_hz"])
    return parser


def main() -> None:
    """Run the map and scene generator from the command line."""
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default="")
    pre_args, _ = pre_parser.parse_known_args()
    config_defaults = load_config_defaults(pre_args.config)
    args = _build_parser(config_defaults).parse_args()
    config = MapConfig(
        output_root=expand_package_path(args.output_root),
        map_name=args.map_name,
        seed=args.seed,
        width=args.width,
        height=args.height,
        resolution=args.resolution,
        obstacle_count=args.obstacle_count,
        min_obstacle_size=args.min_obstacle_size,
        max_obstacle_size=args.max_obstacle_size,
        border_thickness=args.border_thickness,
        corridor_half_width=args.corridor_half_width,
        pixel_threshold=args.pixel_threshold,
        terrain_mode=args.terrain_mode,
        terrain_height_scale=args.terrain_height_scale,
        terrain_negative_height=args.terrain_negative_height,
        terrain_blur_sigma=args.terrain_blur_sigma,
        input_map_image=expand_package_path(args.input_map_image) if args.input_map_image else None,
        input_map_yaml=expand_package_path(args.input_map_yaml) if args.input_map_yaml else None,
        ready_token=args.ready_token,
        dynamic_obstacles_enabled=args.dynamic_obstacles_enabled,
        dynamic_obstacle_count=args.dynamic_obstacle_count,
        dynamic_obstacle_shape=args.dynamic_obstacle_shape,
        dynamic_obstacle_radius=args.dynamic_obstacle_radius,
        dynamic_obstacle_height=args.dynamic_obstacle_height,
        dynamic_obstacle_mass=args.dynamic_obstacle_mass,
        dynamic_obstacle_speed=args.dynamic_obstacle_speed,
        dynamic_obstacle_path_length=args.dynamic_obstacle_path_length,
        dynamic_obstacle_min_robot_distance=args.dynamic_obstacle_min_robot_distance,
        dynamic_obstacle_near_robot_radius=args.dynamic_obstacle_near_robot_radius,
        dynamic_obstacle_map_clearance=args.dynamic_obstacle_map_clearance,
        dynamic_obstacle_replan_rate_hz=args.dynamic_obstacle_replan_rate_hz,
    )
    artifacts = generate_map_assets(config)
    print(f"Map dir: {artifacts.map_dir}")
    print(f"Planner fields: {artifacts.fields_yaml}")
    print(f"Manifest: {artifacts.manifest_yaml}")
    print(f"MuJoCo scene: {artifacts.scene_xml}")
    print(f"Ready file: {artifacts.ready_file}")


if __name__ == "__main__":
    main()
