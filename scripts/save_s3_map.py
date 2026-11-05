#!/usr/bin/env python3
"""Save through the explicit S3 service; archive every raw Point-LIO segment."""
import argparse
import json
from pathlib import Path
import shutil
import time


def pcd_points(path):
    with path.open("rb") as stream:
        for line in stream:
            if line.startswith(b"POINTS "):
                return int(line.split()[1])
            if line.startswith(b"DATA "):
                break
    raise ValueError(f"PCD POINTS missing: {path}")


def pgm_occupied_pixels(path):
    with Path(path).open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError("S3 saver expects binary PGM")
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b"255":
            raise ValueError("S3 saver expects 8-bit PGM")
        pixels = stream.read()
    if len(pixels) != width * height:
        raise ValueError("truncated or oversized PGM")
    return pixels.count(0)


def archive_segments(source, output, since):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    segments = []
    for path in sorted(source.glob("scans*.pcd")):
        if path.stat().st_mtime < since:
            continue
        count = pcd_points(path)
        shutil.copy2(path, output / path.name)
        segments.append(
            {"file": path.name, "points": count, "bytes": path.stat().st_size}
        )
    manifest = {
        "since_unix_s": since,
        "segments": segments,
        "total_points": sum(s["points"] for s in segments),
        "frame": "camera_init",
        "purpose": "raw provenance; transformed map PCD is saved by /rog_map/save_map",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def save_map(prefix, pcd_output, timeout):
    import rclpy
    from ats_rog_map_interfaces.srv import SaveOdomMap

    rclpy.init()
    node = rclpy.create_node("s3_map_saver")
    try:
        client = node.create_client(SaveOdomMap, "/rog_map/save_map")
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/rog_map/save_map unavailable")
        future = client.call_async(SaveOdomMap.Request(output_prefix=str(prefix)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError("save service timeout")
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        print(
            f"saved: occupied={result.occupied_cells} persistent_cells={result.persistent_cells} "
            f"persistent_3d_voxels={result.persistent_voxels} "
            f"raster_occupied_cells={result.raster_occupied_cells}"
        )
        for suffix in (".pgm", ".yaml", ".pcd", ".cells.pcd"):
            if not Path(str(prefix) + suffix).is_file():
                raise RuntimeError(f"save artifact missing: {prefix}{suffix}")
        if pcd_points(Path(str(prefix) + ".pcd")) != result.persistent_voxels:
            raise RuntimeError("3D PCD POINTS and service count differ")
        if pcd_points(Path(str(prefix) + ".cells.pcd")) != result.persistent_cells:
            raise RuntimeError("debug cell PCD POINTS and service count differ")
        if pgm_occupied_pixels(Path(str(prefix) + ".pgm")) != result.raster_occupied_cells:
            raise RuntimeError("PGM occupied pixels and service count differ")
        if pcd_output:
            target = Path(pcd_output)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(prefix) + ".pcd", target)
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix")
    parser.add_argument("--pcd-output")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--archive-source")
    parser.add_argument("--archive-output")
    parser.add_argument("--since", type=float, default=time.time())
    args = parser.parse_args()
    if args.prefix:
        save_map(args.prefix, args.pcd_output, args.timeout)
    if args.archive_source:
        print(
            json.dumps(
                archive_segments(args.archive_source, args.archive_output, args.since)
            )
        )
    if not args.prefix and not args.archive_source:
        parser.error("--prefix or --archive-source required")


if __name__ == "__main__":
    main()
