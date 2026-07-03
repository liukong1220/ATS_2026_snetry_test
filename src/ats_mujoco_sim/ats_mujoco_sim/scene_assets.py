"""Generate MuJoCo swerve scenes from planner map assets."""

from __future__ import annotations

import argparse
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np

from .map_metadata import load_occupancy_image
from .map_metadata import read_map_metadata
from .dynamic_obstacles import DynamicObstacleSceneConfig
from .dynamic_obstacles import build_dynamic_obstacles_xml


def package_root() -> Path:
    """Return the source package root when running from an overlay."""
    return Path(__file__).resolve().parents[1]


def sanitize_name(name: str) -> str:
    """Turn a user-provided map name into a safe file stem."""
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    cleaned = cleaned.strip("_.-")
    return cleaned or "swerve_map"


def _resolve_map_inputs(map_dir: Path) -> tuple[Path, Path]:
    candidates = [
        (map_dir / "global_map.png", map_dir / "global_map.yaml"),
        (map_dir / "map.png", map_dir / "map.yaml"),
    ]
    for image_path, yaml_path in candidates:
        if image_path.exists() and yaml_path.exists():
            return image_path, yaml_path
    raise FileNotFoundError(f"Could not resolve map files under {map_dir}")


def _source_chassis_path() -> Path:
    candidates = [
        package_root() / "models" / "swerve_chassis.xml",
        (
            Path(__file__).resolve().parents[4]
            / "share"
            / "ats_mujoco_sim"
            / "models"
            / "swerve_chassis.xml"
        ),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("Could not resolve swerve_chassis.xml")


def build_terrain_image(
    map_path: Path,
    pixel_threshold: int,
    invert_gray: bool = True,
    blur_sigma: float = 0.0,
    binary: bool = True,
) -> np.ndarray:
    """Convert an occupancy PNG into a MuJoCo heightfield texture."""
    gray = cv2.imread(map_path.as_posix(), cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.size == 0:
        raise RuntimeError(f"Failed to open occupancy image for hfield: {map_path}")

    if binary:
        occupied = gray < int(pixel_threshold)
        low_value = 0 if invert_gray else 255
        high_value = 255 if invert_gray else 0
        terrain = np.full(gray.shape, low_value, dtype=np.uint8)
        terrain[occupied] = high_value
    elif invert_gray:
        terrain = cv2.bitwise_not(gray)
    else:
        terrain = gray.copy()

    if blur_sigma > 0.0:
        terrain = cv2.GaussianBlur(terrain, (0, 0), blur_sigma)
    return terrain


def _occupied_to_world(
    row: int,
    col: int,
    image_height: int,
    resolution: float,
    origin: Tuple[float, float, float],
) -> Tuple[float, float]:
    center_x = origin[0] + (float(col) + 0.5) * resolution
    center_y = origin[1] + (float(image_height) - float(row) - 0.5) * resolution
    return center_x, center_y


def build_box_obstacles_xml(
    occupancy: np.ndarray,
    resolution: float,
    origin: Tuple[float, float, float],
    source_map_path: Path,
    pixel_threshold: int,
    wall_height_m: float = 1.0,
) -> str:
    """Build MuJoCo box geoms from occupied cells."""
    occupied_cells = np.argwhere(occupancy)
    image_height = occupancy.shape[0]
    half_height = wall_height_m * 0.5
    half_cell = 0.5 * resolution
    lines = [
        f"    <!-- Auto-generated from {source_map_path} -->",
        f"    <!-- threshold={pixel_threshold}, wall_height={wall_height_m:.2f}m -->",
        '    <body name="terrain_root" pos="0 0 0">',
    ]

    for index, (row, col) in enumerate(occupied_cells):
        center_x, center_y = _occupied_to_world(row, col, image_height, resolution, origin)
        lines.append(
            "      "
            + (
                f'<geom name="terrain_obstacle_{index:04d}" type="box" '
                f'pos="{center_x:.3f} {center_y:.3f} {half_height:.3f}" '
                f'size="{half_cell:.3f} {half_cell:.3f} {half_height:.3f}" '
                'rgba="0.78 0.76 0.72 1" friction="1 0.1 0.1" '
                'contype="1" conaffinity="1"/>'
            )
        )

    lines.append("    </body>")
    return "\n".join(lines)


def _inject_assets(
    xml_text: str,
    terrain_mode: str,
    map_half_x: float,
    map_half_y: float,
    terrain_height_scale: float,
    terrain_negative_height: float,
) -> str:
    insert = [
        (
            '    <material name="terrain_material" rgba="0.62 0.62 0.60 1" '
            'reflectance="0.0" specular="0.0" shininess="0.0" emission="0.15"/>'
        ),
    ]
    if terrain_mode == "hfield":
        insert.append(
            '    <hfield name="terrain_hfield" '
            f'size="{map_half_x:.3f} {map_half_y:.3f} '
            f'{terrain_height_scale:.3f} {terrain_negative_height:.3f}" '
            'file="terrain.png"/>'
        )
    return xml_text.replace(
        "  </asset>",
        "\n".join(insert) + "\n  </asset>",
        1,
    )


def _inject_worldbody(xml_text: str, terrain_xml: str) -> str:
    lines = xml_text.splitlines()
    for index, line in enumerate(lines):
        if "<geom name=\"floor\"" in line:
            lines[index] = f"{line}\n{terrain_xml}"
            return "\n".join(lines) + "\n"
    raise RuntimeError("Could not find floor geom in swerve_chassis.xml")


def _replace_floor_geom(
    xml_text: str,
    terrain_mode: str,
    map_center_x: float,
    map_center_y: float,
    map_half_x: float,
    map_half_y: float,
) -> str:
    """Resize the visual floor and avoid hfield/floor z-fighting."""
    if terrain_mode == "hfield":
        floor_xml = (
            f'    <geom name="floor" type="plane" '
            f'pos="{map_center_x:.3f} {map_center_y:.3f} -0.050" '
            f'size="{map_half_x:.3f} {map_half_y:.3f} 0.05" '
            'material="grid" contype="0" conaffinity="0"/>'
        )
    else:
        floor_xml = (
            f'    <geom name="floor" type="plane" '
            f'pos="{map_center_x:.3f} {map_center_y:.3f} 0" '
            f'size="{map_half_x:.3f} {map_half_y:.3f} 0.05" '
            'material="grid"/>'
        )

    return xml_text.replace(
        '    <geom name="floor" type="plane" size="3 3 0.05" material="grid"/>',
        floor_xml,
        1,
    )


def build_scene_xml(
    scene_name: str,
    terrain_mode: str,
    occupancy: np.ndarray,
    resolution: float,
    origin: Tuple[float, float, float],
    source_map_path: Path,
    pixel_threshold: int,
    terrain_height_scale: float = 0.15,
    terrain_negative_height: float = 0.03,
    wall_height_m: float = 1.0,
    dynamic_obstacles: DynamicObstacleSceneConfig | None = None,
) -> str:
    """Inject terrain into the base swerve chassis MuJoCo XML."""
    terrain_mode = terrain_mode.lower().strip()
    if terrain_mode not in ("hfield", "boxes", "dynamic"):
        raise ValueError(f"Unsupported terrain_mode: {terrain_mode}")

    chassis_xml = _source_chassis_path().read_text(encoding="utf-8")
    chassis_xml = chassis_xml.replace(
        '<mujoco model="four_swerve_chassis">',
        f'<mujoco model="{scene_name}">',
        1,
    )

    map_half_x = 0.5 * float(occupancy.shape[1]) * resolution
    map_half_y = 0.5 * float(occupancy.shape[0]) * resolution
    map_center_x = origin[0] + map_half_x
    map_center_y = origin[1] + map_half_y
    chassis_xml = _replace_floor_geom(
        chassis_xml,
        terrain_mode,
        map_center_x,
        map_center_y,
        map_half_x,
        map_half_y,
    )
    mode_for_assets = "hfield" if terrain_mode == "hfield" else "boxes"
    chassis_xml = _inject_assets(
        chassis_xml,
        mode_for_assets,
        map_half_x,
        map_half_y,
        terrain_height_scale,
        terrain_negative_height,
    )

    if terrain_mode == "hfield":
        terrain_xml = (
            '    <geom name="terrain" type="hfield" hfield="terrain_hfield" '
            f'pos="{map_center_x:.3f} {map_center_y:.3f} 0" '
            'material="terrain_material" condim="3" friction="1 0.1 0.1"/>'
        )
    else:
        terrain_xml = build_box_obstacles_xml(
            occupancy=occupancy,
            resolution=resolution,
            origin=origin,
            source_map_path=source_map_path,
            pixel_threshold=pixel_threshold,
            wall_height_m=wall_height_m,
        )
    if dynamic_obstacles is not None:
        dynamic_xml = build_dynamic_obstacles_xml(
            dynamic_obstacles,
            map_center_x=map_center_x,
            map_center_y=map_center_y,
            map_half_x=map_half_x,
            map_half_y=map_half_y,
        )
        if dynamic_xml:
            terrain_xml = terrain_xml + "\n" + dynamic_xml
    return _inject_worldbody(chassis_xml, terrain_xml)


@dataclass(frozen=True)
class SceneArtifacts:
    scene_out: Path
    terrain_image: Path | None
    model_dir: Path


def generate_scene_assets(
    map_dir: Path,
    model_dir: Path,
    scene_name: str,
    pixel_threshold: int = 230,
    terrain_mode: str = "hfield",
    terrain_height_scale: float = 0.15,
    terrain_negative_height: float = 0.03,
    terrain_blur_sigma: float = 0.0,
    wall_height_m: float = 1.0,
    dynamic_obstacles: DynamicObstacleSceneConfig | None = None,
) -> SceneArtifacts:
    """Generate scene XML and terrain images under a model directory."""
    map_dir = Path(map_dir)
    model_dir = Path(model_dir)
    scene_name = sanitize_name(scene_name)
    model_dir.mkdir(parents=True, exist_ok=True)

    source_map_path, map_yaml_path = _resolve_map_inputs(map_dir)
    resolution, origin = read_map_metadata(map_yaml_path)
    occupancy = load_occupancy_image(source_map_path, pixel_threshold)

    shutil.copy2(source_map_path, model_dir / "map.png")
    if (map_dir / "global_map.png").exists():
        shutil.copy2(map_dir / "global_map.png", model_dir / "global_map.png")

    terrain_image_path = model_dir / "terrain.png"
    terrain = build_terrain_image(
        source_map_path,
        pixel_threshold=pixel_threshold,
        invert_gray=True,
        blur_sigma=terrain_blur_sigma,
    )
    cv2.imwrite(terrain_image_path.as_posix(), terrain)

    scene_out = model_dir / f"{scene_name}.xml"
    scene_out.write_text(
        build_scene_xml(
            scene_name=scene_name,
            terrain_mode=terrain_mode,
            occupancy=occupancy,
            resolution=resolution,
            origin=origin,
            source_map_path=source_map_path,
            pixel_threshold=pixel_threshold,
            terrain_height_scale=terrain_height_scale,
            terrain_negative_height=terrain_negative_height,
            wall_height_m=wall_height_m,
            dynamic_obstacles=dynamic_obstacles,
        ),
        encoding="utf-8",
    )

    return SceneArtifacts(
        scene_out=scene_out,
        terrain_image=terrain_image_path,
        model_dir=model_dir,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a swerve MuJoCo scene from a map directory."
    )
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--scene-name", default="swerve")
    parser.add_argument("--pixel-threshold", type=int, default=230)
    parser.add_argument("--terrain-mode", choices=["hfield", "boxes", "dynamic"], default="hfield")
    parser.add_argument("--terrain-height-scale", type=float, default=0.15)
    parser.add_argument("--terrain-negative-height", type=float, default=0.03)
    parser.add_argument("--terrain-blur-sigma", type=float, default=0.0)
    parser.add_argument("--wall-height-m", type=float, default=1.0)
    parser.add_argument("--dynamic-obstacles-enabled", action="store_true")
    parser.add_argument("--dynamic-obstacle-count", type=int, default=0)
    parser.add_argument("--dynamic-obstacle-shape", default="cylinder")
    parser.add_argument("--dynamic-obstacle-radius", type=float, default=0.25)
    parser.add_argument("--dynamic-obstacle-height", type=float, default=1.0)
    parser.add_argument("--dynamic-obstacle-mass", type=float, default=20.0)
    parser.add_argument("--dynamic-obstacle-speed", type=float, default=0.4)
    parser.add_argument("--dynamic-obstacle-path-length", type=float, default=6.0)
    return parser


def main() -> None:
    """Run the scene-asset generator from the command line."""
    args = _build_parser().parse_args()
    artifacts = generate_scene_assets(
        map_dir=Path(args.map_dir).expanduser(),
        model_dir=Path(args.model_dir).expanduser(),
        scene_name=args.scene_name,
        pixel_threshold=args.pixel_threshold,
        terrain_mode=args.terrain_mode,
        terrain_height_scale=args.terrain_height_scale,
        terrain_negative_height=args.terrain_negative_height,
        terrain_blur_sigma=args.terrain_blur_sigma,
        wall_height_m=args.wall_height_m,
        dynamic_obstacles=DynamicObstacleSceneConfig(
            enabled=args.dynamic_obstacles_enabled,
            count=args.dynamic_obstacle_count,
            shape=args.dynamic_obstacle_shape,
            radius=args.dynamic_obstacle_radius,
            height=args.dynamic_obstacle_height,
            mass=args.dynamic_obstacle_mass,
            speed=args.dynamic_obstacle_speed,
            path_length=args.dynamic_obstacle_path_length,
        ),
    )
    print(f"MuJoCo scene: {artifacts.scene_out}")


if __name__ == "__main__":
    main()
