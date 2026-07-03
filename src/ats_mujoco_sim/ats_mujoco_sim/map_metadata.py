"""Helpers for planner occupancy-map metadata."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Tuple

import cv2


def read_map_metadata(yaml_path: Path) -> Tuple[float, Tuple[float, float, float]]:
    """Read resolution and origin from a ROS occupancy-map YAML file."""
    resolution = None
    origin = None
    with yaml_path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                origin = tuple(float(v) for v in ast.literal_eval(line.split(":", 1)[1].strip()))

    if resolution is None or origin is None:
        raise RuntimeError(f"Failed to parse map metadata from {yaml_path}")
    return resolution, origin


def load_occupancy_image(map_path: Path, pixel_threshold: int = 230):
    """Load an occupancy image as a boolean obstacle mask."""
    gray = cv2.imread(map_path.expanduser().resolve().as_posix(), cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.size == 0:
        raise RuntimeError(f"Failed to open occupancy image: {map_path}")
    return gray < pixel_threshold


def map_bounds_from_files(
    image_path: str,
    yaml_path: str,
    default_width: float,
    default_height: float,
    default_resolution: float,
) -> tuple[float, float, float, float, float]:
    """Return map bounds from image/YAML inputs or generated-map defaults."""
    image_path = str(image_path).strip()
    yaml_path = str(yaml_path).strip()

    if image_path and yaml_path:
        image_path = Path(image_path).expanduser().resolve()
        yaml_path = Path(yaml_path).expanduser().resolve()
        image = cv2.imread(image_path.as_posix(), cv2.IMREAD_GRAYSCALE)
        if image is None or image.size == 0:
            raise RuntimeError(f"Failed to open occupancy image: {image_path}")
        resolution, origin = read_map_metadata(yaml_path)
        width = image.shape[1]
        height = image.shape[0]
        x_lower = float(origin[0])
        x_upper = float(origin[0]) + float(width) * float(resolution)
        y_lower = float(origin[1])
        y_upper = float(origin[1]) + float(height) * float(resolution)
        return x_lower, x_upper, y_lower, y_upper, float(resolution)

    width = float(default_width)
    height = float(default_height)
    resolution = float(default_resolution)
    x_lower = -0.5 * width * resolution
    x_upper = 0.5 * width * resolution
    y_lower = -0.5 * height * resolution
    y_upper = 0.5 * height * resolution
    return x_lower, x_upper, y_lower, y_upper, resolution
