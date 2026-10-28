#!/usr/bin/env python3
"""Offline regression for scripts/generate_rmuc_2027_map.py (no ROS, no MuJoCo).

Rule tests cut tiny hand-made meshes whose answer is known.  Drift tests rebuild the committed
map from the committed field mesh and compare it byte for byte.  A map that no longer matches its
generator, or a rule change that closes a tunnel or the highland ramp, fails here.

    python3 -I scripts/test_generate_rmuc_2027_map.py
"""

import dataclasses
import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generate_rmuc_2027_map", WORKSPACE / "scripts/generate_rmuc_2027_map.py")
GEN = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GEN          # dataclasses look the module up by name
SPEC.loader.exec_module(GEN)
MAP_DIR = WORKSPACE / "src/ats_sentry_bringup/map"
RES = 0.05


# ---- hand-made meshes.  Vertical rays never see vertical faces, so a solid is a top and a bottom.
def _quad(x0, x1, y0, y1, heights, up):
    p = [(x0, y0, heights[0]), (x1, y0, heights[1]), (x1, y1, heights[2]), (x0, y1, heights[3])]
    triangles = [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
    return triangles if up else [t[::-1] for t in triangles]


def solid(x0, x1, y0, y1, z_bottom, z_top):
    return _quad(x0, x1, y0, y1, [z_top] * 4, True) + _quad(x0, x1, y0, y1, [z_bottom] * 4, False)


def wedge(x0, x1, y0, y1, z_bottom, z_low, z_high):
    """Top rises along +x from z_low to z_high."""
    return (_quad(x0, x1, y0, y1, [z_low, z_high, z_high, z_low], True)
            + _quad(x0, x1, y0, y1, [z_bottom] * 4, False))


def undulating_road(x0, x1, y0, y1, base, amplitude, wavelength, step=0.0125):
    """Sine ridges along x, valleys at `base`, peak-to-valley 2 * amplitude, on top of the ground."""
    xs = np.arange(x0, x1 + step / 2, step)
    wave = lambda x: base + amplitude * (1.0 - math.cos(2.0 * math.pi * (x - x0) / wavelength))
    triangles = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        triangles += _quad(xa, xb, y0, y1, [wave(xa), wave(xb), wave(xb), wave(xa)], True)
    return triangles + _quad(x0, xs[-1], y0, y1, [base] * 4, False)


def small_rules(**changes):
    return dataclasses.replace(GEN.Rules(), width_cells=40, height_cells=32, border_cells=0,
                               seeds=((0.25, 0.8),), **changes)


def corridor(x0, x1, y0, y1):
    return {"corridor_polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
            "entry": [0.25, 0.8, 0.0], "exit": [0.25, 0.8, 0.0]}


def build(solids, rules, corridors=()):
    triangles = np.array([t for s in solids for t in s], dtype=np.float64)
    return GEN.build_map_from_keys(GEN.rasterize([triangles], rules), list(corridors), rules)


class RuleTest(unittest.TestCase):
    """Map 2.0 x 1.6 m on a 0.10 m ground slab; every feature sits on top of the slab."""

    @staticmethod
    def scene(ramp_degrees=12.0, curb=None):
        top = 0.10 + 0.6 * math.tan(math.radians(ramp_degrees))
        solids = [solid(0, 2, 0, 1.6, 0, 0.10),
                  solid(0.30, 0.50, 0.20, 0.40, 0.10, 0.25),          # 0.15 m pad
                  solid(0.30, 0.60, 0.60, 0.80, 0.35, 0.40),          # roof, 0.25 m clear
                  solid(0.30, 0.60, 0.90, 1.10, 0.30, 0.35),          # roof, 0.20 m clear
                  wedge(1.00, 1.60, 0, 1.6, 0.10, 0.10, top),
                  solid(1.60, 1.90, 0, 1.6, 0.10, top),
                  solid(1.92, 2.00, 0, 1.6, 0.10, 0.60)]              # wall, half a cell into 1.90-1.95
        if curb:
            solids.append(solid(0.70, 0.90, 0, 1.6, 0.10, 0.10 + curb))
        return solids

    def occupied(self, **kwargs):
        return build(self.scene(**kwargs), small_rules())["occupied"]

    def test_pad_taller_than_a_step_is_an_obstacle_and_touching_solids_are_one_surface(self):
        occ = self.occupied()
        self.assertTrue(occ[4:8, 6:10].all())
        self.assertFalse(occ[3, 6:10].any() or occ[8, 6:10].any() or occ[4:8, 5].any() or occ[4:8, 10].any())

    def test_roof_is_an_obstacle_only_below_the_vehicle_height(self):
        occ = self.occupied()
        self.assertFalse(occ[12:16, 6:12].any(), "0.25 m under the roof must stay free for a 0.242 m vehicle")
        self.assertTrue(occ[18:22, 6:12].all(), "0.20 m under the roof is too low")
        self.assertFalse(occ[17, 6:12].any() or occ[22, 6:12].any())

    def test_twelve_degree_ramp_is_climbable_and_a_twenty_five_degree_one_is_not(self):
        result = build(self.scene(), small_rules())
        self.assertFalse(result["occupied"][:, 20:38].any())
        self.assertAlmostEqual(float(result["floor"][16, 25]), 0.10 + 0.275 * math.tan(math.radians(12)), delta=0.01)
        # The first two columns are the toe, which is no steeper than a 5 cm step there.
        self.assertTrue(self.occupied(ramp_degrees=25.0)[:, 21:38].all())

    def test_undulating_road_is_drivable_but_tall_waves_are_not(self):
        """RMUC undulating road: 6.7 cm peak-to-valley every 24.5 cm, 41 deg on the steepest flank.

        Judged per 5 cm cell those flanks look like walls; the chassis sees about 12 deg over its
        0.32 m wheelbase.  Waves three times as tall are an obstacle.
        """
        ground = solid(0, 2, 0, 1.6, 0, 0.10)
        for amplitude, drivable in ((0.0335, True), (0.10, False)):
            occ = build([ground, undulating_road(0.80, 1.52, 0, 1.6, 0.10, amplitude, 0.245)], small_rules())["occupied"]
            band = occ[2:30, 18:29]          # x 0.90-1.45 m, inside the waves
            assert (not band.any()) if drivable else band.mean() > 0.9, (amplitude, float(band.mean()))

    def test_step_height_limit(self):
        self.assertFalse(self.occupied(curb=0.03)[:, 14:18].any())
        self.assertTrue(self.occupied(curb=0.06)[:, 14:18].all())

    def test_wall_cell_with_half_coverage_is_occupied(self):
        occ = self.occupied()
        self.assertTrue(occ[:, 38:40].all())
        self.assertFalse(occ[:, 37].any())

    def test_border_ring_is_occupied_when_requested(self):
        rules = dataclasses.replace(small_rules(), border_cells=1)
        occ = build(self.scene(), rules)["occupied"]
        self.assertTrue(occ[0].all() and occ[-1].all() and occ[:, 0].all() and occ[:, -1].all())


class GapTest(unittest.TestCase):
    """A 0.795 m channel between two walls, the width of the real tunnels."""

    @staticmethod
    def free_rows(wall_b, roof=False, corridors=()):
        solids = [solid(0, 2, 0.50, wall_b, 0, 0.10), solid(0, 2, 0, 0.50, 0, 0.60),
                  solid(0, 2, wall_b, 1.60, 0, 0.60)]
        if roof:
            solids.append(solid(0.90, 1.10, 0.50, wall_b, 0.30, 0.35))
        occ = build(solids, small_rules(), corridors)["occupied"]
        return [row for row in range(32) if not occ[row, 10]], occ

    def test_795_mm_channel_is_sixteen_cells(self):
        rows, _ = self.free_rows(1.295)
        self.assertEqual((rows[0], rows[-1], len(rows)), (10, 25, 16))

    def test_a_quarter_covered_cell_stays_free_and_a_half_covered_one_does_not(self):
        self.assertEqual(len(self.free_rows(1.2875)[0]), 16)
        self.assertEqual(len(self.free_rows(1.28)[0]), 15)

    def test_a_low_roof_closes_the_channel_unless_it_is_inside_a_corridor(self):
        _, closed = self.free_rows(1.295, roof=True)
        self.assertTrue(closed[10:26, 18:22].all())
        _, open_ = self.free_rows(1.295, roof=True, corridors=[corridor(0.9, 1.1, 0.5, 1.295)])
        self.assertFalse(open_[10:26, 18:22].any())


class CommittedMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rules = GEN.Rules()
        cls.pgm_bytes = (MAP_DIR / "rmuc_2027.pgm").read_bytes()
        cls.occupied = GEN.decode_pgm(cls.pgm_bytes)
        cls.tunnels = GEN.load_tunnels(GEN.DEFAULT_TUNNELS)
        cls.manifest = json.loads((MAP_DIR / "rmuc_2027.manifest.json").read_text())

    def cell(self, x, y):
        return int(y / RES), int(x / RES)

    def test_yaml_describes_the_image(self):
        meta = yaml.safe_load((MAP_DIR / "rmuc_2027.yaml").read_text())
        self.assertEqual(meta["image"], "rmuc_2027.pgm")
        self.assertEqual((meta["resolution"], meta["origin"], meta["negate"]), (0.05, [0.0, 0.0, 0.0], 0))
        self.assertEqual((meta["occupied_thresh"], meta["free_thresh"]), (0.65, 0.196))
        self.assertTrue(self.pgm_bytes.startswith(b"P5\n560 320\n255\n"))
        pixels = np.frombuffer(self.pgm_bytes[len(b"P5\n560 320\n255\n"):], dtype=np.uint8)
        self.assertEqual(set(np.unique(pixels).tolist()), {GEN.OCCUPIED_PIXEL, GEN.FREE_PIXEL})

    def test_committed_files_match_their_manifest(self):
        self.assertEqual(GEN.sha256(self.pgm_bytes), self.manifest["outputs"]["rmuc_2027.pgm"])
        self.assertEqual(GEN.sha256(MAP_DIR / "rmuc_2027.yaml"), self.manifest["outputs"]["rmuc_2027.yaml"])
        mesh = WORKSPACE / self.manifest["inputs"]["mesh"]
        self.assertEqual(GEN.sha256(mesh), self.manifest["inputs"]["mesh_sha256"], "field mesh changed: regenerate the map")
        tunnels = WORKSPACE / self.manifest["inputs"]["tunnels"]
        self.assertEqual(GEN.sha256(tunnels), self.manifest["inputs"]["tunnels_sha256"], "tunnels yaml changed: regenerate the map")
        self.assertEqual(self.manifest["rules"], json.loads(json.dumps(dataclasses.asdict(self.rules))))

    def test_map_is_exactly_what_the_generator_makes_from_the_field_mesh(self):
        result = GEN.build_map(GEN.DEFAULT_MESH, GEN.DEFAULT_TUNNELS, self.rules)
        regenerated = GEN.encode_pgm(result["occupied"], self.rules)
        drift = int((GEN.decode_pgm(regenerated) != self.occupied).sum())
        self.assertEqual(regenerated, self.pgm_bytes, f"{drift} cells differ from the generator output")

    def test_field_is_point_symmetric_so_the_image_is_not_flipped(self):
        inner = (slice(1, -1), slice(1, -1))
        agreement = float((self.occupied == self.occupied[::-1, ::-1])[inner].mean())
        self.assertGreater(agreement, 0.995)
        # An x or y flip of a point-symmetric field leaves it symmetric; the hexagon pads and the
        # spawn points say which way is which.
        for x, y in ((6.6, 8.0), (21.4, 8.0), (14.0, 8.0)):
            self.assertTrue(self.occupied[self.cell(x, y)], (x, y))
        for x, y in ((5.0, 8.0), (23.0, 8.0), (14.0, 11.6)):
            self.assertFalse(self.occupied[self.cell(x, y)], (x, y))
        # All the points above sit on y = 8 or x = 14, where an x or y flip of this point-symmetric
        # field changes nothing.  These two blocks beside the pill differ under every flip.
        def block(x, y):
            row, col = self.cell(x, y)
            return self.occupied[row - 2:row + 3, col - 2:col + 3]
        self.assertTrue(block(14.78, 8.38).all(), "landmark that is occupied only in the correct orientation")
        self.assertFalse(block(14.88, 7.68).any(), "landmark that is free only in the correct orientation")

    def test_encode_pgm_puts_map_row_zero_on_the_last_file_row(self):
        small = dataclasses.replace(self.rules, width_cells=4, height_cells=3)
        occupied = np.zeros((3, 4), dtype=bool)
        occupied[0, 1] = True                       # y = 0 row, second column
        data = GEN.encode_pgm(occupied, small)
        header = b"P5\n4 3\n255\n"
        pixels = np.frombuffer(data[len(header):], dtype=np.uint8).reshape(3, 4)
        self.assertEqual(int(pixels[2, 1]), GEN.OCCUPIED_PIXEL)         # last file row
        self.assertEqual(int((pixels == GEN.OCCUPIED_PIXEL).sum()), 1)
        self.assertTrue(GEN.decode_pgm(data)[0, 1])

    def test_mesh_cracks_stay_bounded(self):
        """Up/down faces of the 'closed' field mesh balance in all but ~1.5 % of the columns."""
        result = GEN.build_map(GEN.DEFAULT_MESH, GEN.DEFAULT_TUNNELS, self.rules)
        self.assertLess(result["layers"]["unbalanced_columns"], 0.03)

    def test_the_field_sides_are_not_drawn_as_solid_walls(self):
        self.assertLess(float(self.occupied[:, :int(3.1 / RES)].mean()), 0.6)
        self.assertLess(float(self.occupied[:, int(24.9 / RES):].mean()), 0.6)
        ring = np.r_[self.occupied[0], self.occupied[-1], self.occupied[:, 0], self.occupied[:, -1]]
        self.assertTrue(ring.all(), "the one-cell field boundary must be occupied")

    def test_west_highland_and_east_are_connected_for_a_robot_sized_footprint(self):
        points = {"west": (5.0, 8.0), "highland": (14.0, 11.6), "east": (23.0, 8.0)}
        for radius in (0.0, 0.20, 0.30):
            labels = GEN.free_labels(self.occupied, int(round(radius / RES)))
            ids = {name: int(labels[self.cell(*xy)]) for name, xy in points.items()}
            self.assertTrue(all(i >= 0 for i in ids.values()), (radius, ids))
            self.assertEqual(len(set(ids.values())), 1, f"regions split at {radius} m erosion: {ids}")

    def test_highland_is_reachable_without_the_tunnels(self):
        closed = self.occupied.copy()
        for x0, x1, y0, y1 in GEN.corridor_boxes(self.tunnels):
            closed[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = True
        labels = GEN.free_labels(closed, int(round(0.30 / RES)))
        self.assertGreaterEqual(int(labels[self.cell(14.0, 11.6)]), 0)
        self.assertEqual(int(labels[self.cell(5.0, 8.0)]), int(labels[self.cell(14.0, 11.6)]))
        self.assertEqual(int(labels[self.cell(23.0, 8.0)]), int(labels[self.cell(14.0, 11.6)]))

    def test_the_two_undulating_road_sections_are_free_and_connected(self):
        """The 2.4 x 2.1 m corrugated plateaus (x 5.3-7.7 / mirrored) are drivable, not obstacles."""
        sections = {"lower-left": (5.3, 7.7, 0.7, 2.5), "upper-right": (20.3, 22.7, 13.5, 15.3)}
        for name, (x0, x1, y0, y1) in sections.items():
            block = self.occupied[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)]
            assert float(block.mean()) < 0.02, f"{name} undulating road is drawn as an obstacle"
        labels = GEN.free_labels(self.occupied, int(round(0.30 / RES)))
        spawn = int(labels[self.cell(5.0, 8.0)])
        assert spawn >= 0
        for point in ((6.5, 1.5), (21.5, 14.5)):
            assert int(labels[self.cell(*point)]) == spawn, point

    def test_both_tunnels_keep_the_795_mm_channel_under_and_beside_the_lintel(self):
        widths = GEN.tunnel_widths(self.occupied, self.tunnels, self.rules)
        self.assertEqual(set(widths), {"tunnel_B", "tunnel_B_mirror"})
        for name, info in widths.items():
            self.assertGreaterEqual(info["min_width_m"], 0.795, (name, info))
            self.assertGreaterEqual(info["lintel_min_width_m"], 0.795, (name, info))
            self.assertLessEqual(info["min_width_m"], 0.95, f"{name} channel is suspiciously wide: {info}")

    def test_tunnel_axis_is_free_end_to_end(self):
        for tunnel in self.tunnels:
            (x0, _, _, _), axis = GEN.corridor_boxes([tunnel])[0], int(tunnel["entry"][1] / RES)
            x1 = GEN.corridor_boxes([tunnel])[0][1]
            row = self.occupied[axis, int(x0 / RES):int(x1 / RES) + 1]
            self.assertFalse(row.any(), tunnel["name"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
