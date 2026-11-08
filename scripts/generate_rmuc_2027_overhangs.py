#!/usr/bin/env python3
"""Generate simulation-only multilayer collision assets from the R1 audit.

The official STL remains authoritative. Unbalanced columns retain their highest
surface (solid fallback) and are listed in the generated diagnostics.
No PGM or hand-authored tunnel rectangle is used to remove collision geometry.
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from generate_rmuc_2027_map import Rules, Z_BITS, Z_LEVELS, Z_PER_METRE, build_layers, iter_triangles, rasterize, sha256

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/sim/ats_mujoco_sim'
sys.path.insert(0, str(SIM))
from ats_mujoco_sim.rmuc_overhangs import partition_layers, roof_xml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, default=ROOT / '2027_map/RMUC2027_V1.0.0.STL')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--ray-cache', type=Path, help='validated offline ray cache (not navigation input)')
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text())
    if sha256(args.mesh) != audit['mesh_sha256']:
        raise ValueError('official mesh does not match audited input')
    for source in ('audit_rmuc_2027_overhangs.py', 'generate_rmuc_2027_map.py'):
        path = Path(__file__).with_name(source)
        if sha256(path) != audit['source_hashes'][str(path)]:
            raise ValueError(f'audit implementation changed: {source}; re-audit first')
    rules = Rules(**audit['rules'])
    width, height, step = rules.width_cells, rules.height_cells, rules.resolution
    if rules.supersample != 1:
        raise ValueError('expected audit at one ray per cell')
    all_cells = np.asarray(audit['cells'], dtype=float)[:, :4]
    valid = (all_cells[:, 3] > all_cells[:, 2]) & (all_cells[:, 2] > all_cells[:, 1])
    # Coincident opposite faces can produce a zero-thickness roof in R1.
    # They are explicitly unresolved, never converted to a zero-size/free geom.
    unresolved = all_cells[~valid].tolist()
    cells = all_cells[valid]
    # Rasterize all official faces, not the decimated visual mesh. Keep the
    # highest surface unless an independently bounded audited cavity proves a floor.
    cache_identity = json.dumps([audit['mesh_sha256'], audit['mesh_scale'], audit['rules'],
                                 sha256(Path(__file__).with_name('generate_rmuc_2027_map.py'))], sort_keys=True)
    if args.ray_cache and args.ray_cache.is_file():
        with np.load(args.ray_cache, allow_pickle=False) as cache:
            if str(cache['identity']) != cache_identity:
                raise ValueError('ray cache input/hash mismatch')
            keys = cache['keys']
    else:
        keys = rasterize(iter_triangles(args.mesh, audit['mesh_scale']), rules)
        if args.ray_cache:
            args.ray_cache.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(args.ray_cache, identity=cache_identity, keys=keys)
    # An unbalanced column need not invalidate a locally closed roof: e.g.
    # tunnel_B has an extra downward sheet BELOW its 1-mm floor plate. Winding
    # above the floor is still zero and the roof is a closed down/up pair.
    # Recover only that independently bounded interval; retain a full ledger.
    layers = build_layers(keys, rules, [])
    columns, heights = layers['column'], layers['z']
    existing = set(cells[:, 0].astype(int).tolist())
    recovered = []
    candidates = np.flatnonzero(layers['exposed'] & np.isfinite(layers['clearance'])
                               & (layers['clearance'] > rules.interface_gap))
    for i in candidates:
        column = int(columns[i])
        if column in existing:
            continue
        bottom = heights[i] + layers['clearance'][i]
        end = np.searchsorted(columns, column, side='right')
        tops = np.flatnonzero(layers['exposed'][i+1:end] & (heights[i+1:end] > bottom + rules.interface_gap)) + i + 1
        if len(tops):
            recovered.append([column, float(heights[i]), float(bottom), float(heights[tops[0]])])
            existing.add(column)
    if recovered:
        cells = np.vstack([cells, recovered])
    cells = cells[np.argsort(cells[:, 0])]
    plates = partition_layers(cells, width, step)
    up = keys[(keys & 1) != 0]
    missing_up = np.setdiff1d(np.arange(width * height), up >> (Z_BITS + 1))
    col = keys >> (Z_BITS + 1)
    z = ((keys >> 1) & (Z_LEVELS - 1)) / Z_PER_METRE
    missing_any = np.setdiff1d(np.arange(width * height), col)
    # The supplied official mesh has four true no-surface rays, confirmed by
    # independent exact-coordinate probes. Keep them explicitly unresolved;
    # zero is only the hfield's global lower datum, NOT evidence of free space
    # or verified support. This simulation asset is never a navigation map.
    coincident = (np.diff(keys) == 1) & ((keys[:-1] & 1) == 0)
    field = np.zeros(width * height)
    np.maximum.at(field, col, z)
    ids = cells[:, 0].astype(int)
    field[ids] = cells[:, 1]
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    png = output / 'rmuc_2027_height.png'
    # MuJoCo samples PNG nodes including both edges. The scene half extents are
    # (N-1)*step/2, so audit cell centres and physical nodes coincide exactly.
    elevation = float(field.max())
    normalized = field.reshape(height, width) / elevation
    image = np.rint(normalized * 255).astype(np.uint8)
    Image.fromarray(np.flipud(image)).save(png)
    # PNG is a preview only: full-height 8-bit samples would quantize the field
    # by 15 mm. MuJoCo's native float hfield keeps sub-mm official elevations.
    binary = output / 'rmuc_2027_height.bin'
    binary.write_bytes(struct.pack('<ii', height, width) + normalized.astype('<f4').tobytes())
    roofs = output / 'rmuc_2027_roof_boxes.xml'
    roofs.write_text(roof_xml(plates))
    manifest = dict(schema_version=1, purpose='simulation_only_not_navigation_input',
                    mesh_sha256=audit['mesh_sha256'], audit_sha256=sha256(args.audit),
                    generator_sha256=sha256(Path(__file__)),
                    library_sha256=sha256(SIM / 'ats_mujoco_sim/rmuc_overhangs.py'),
                    resolution=step, width=width, height=height,
                    hfield_half_extents=[(width-1)*step/2, (height-1)*step/2],
                    hfield_elevation=elevation,
                    g_box=0., g_box_reason='No positive fill-equivalence bound established for complete robot; preserve every audited cavity, including D1 and D4.',
                    diagnostics=audit['diagnostics'],
                    columns_without_upward_surface=missing_up.tolist(),
                    columns_without_any_surface=missing_any.tolist(),
                    no_surface_policy='unresolved void at global lower datum; not verified support or free-space evidence',
                    coincident_opposite_face_columns=np.unique(col[:-1][coincident]).tolist(),
                    unresolved_layer_records=unresolved,
                    locally_closed_recovered_records=recovered,
                    unresolved_policy='highest official surface regardless of orientation; no-surface columns explicitly unresolved, never navigation free-space evidence',
                    cells_preserved=len(cells), boxes=len(plates),
                    single_cell_boxes=sum(p['cells'] == 1 for p in plates),
                    maximum_bottom_error_m=max(p['bottom_error_m'] for p in plates),
                    maximum_top_error_m=max(p['top_error_m'] for p in plates),
                    outputs={png.name:sha256(png), binary.name:sha256(binary), roofs.name:sha256(roofs)},
                    plates=plates)
    (output / 'overhangs_2027.generated.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('plates', 'locally_closed_recovered_records')}, indent=2))
    print(f'locally closed recovered records: {len(recovered)}')


if __name__ == '__main__':
    main()
