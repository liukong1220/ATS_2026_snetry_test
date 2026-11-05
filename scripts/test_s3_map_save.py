#!/usr/bin/env python3
"""Persistence must include all current-session PCD chunks and exclude old runs."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    "s3_save", Path(__file__).with_name("save_s3_map.py")
)
save = importlib.util.module_from_spec(spec)
spec.loader.exec_module(save)


class SegmentArchive(unittest.TestCase):
    def test_pgm_count_is_independent_of_three_dimensional_point_count(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.pgm"
            path.write_bytes(b"P5\n3 2\n255\n" + bytes([0, 254, 205, 0, 205, 254]))
            self.assertEqual(save.pgm_occupied_pixels(path), 2)
            path.write_bytes(b"P5\n3 2\n255\n" + bytes([0, 254]))
            with self.assertRaises(ValueError):
                save.pgm_occupied_pixels(path)

    def test_all_segments_and_shutdown_tail_are_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            for name, mtime in [
                ("scans_1.pcd", 10),
                ("scans_2.pcd", 11),
                ("scans_shutdown.pcd", 12),
                ("scans_old.pcd", 1),
            ]:
                path = source / name
                path.write_text("POINTS 3\nDATA ascii\n1 2 3\n1 2 4\n1 2 5\n")
                os.utime(path, (mtime, mtime))
            manifest = save.archive_segments(source, root / "archive", 9.0)
            self.assertEqual(len(manifest["segments"]), 3)
            self.assertEqual(manifest["total_points"], 9)
            self.assertFalse((root / "archive/scans_old.pcd").exists())
            self.assertEqual(
                json.loads((root / "archive/manifest.json").read_text()), manifest
            )

    def test_missing_chunks_are_reported_as_empty_not_stale_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(
                save.archive_segments(root / "missing", root / "out", 0)["segments"], []
            )


if __name__ == "__main__":
    unittest.main()
