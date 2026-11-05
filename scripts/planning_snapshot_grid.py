#!/usr/bin/env python3
"""Occupancy view of ``/rog_map_adapter/planning_snapshot`` for test tools.

M4 removed ``/rc_esdf/planning_grid``; the numeric planning snapshot is the only
planning-map contract. Tools that used to read the grid convert each snapshot
with :func:`snapshot_to_grid`, which keeps the old grid semantics:

* ``header.frame_id`` is the snapshot frame and ``header.stamp`` its
  ``source_stamp`` (the fused-grid stamp the old topic carried);
* ``data`` is the snapshot occupancy (-1 unknown, 0..99 risk, >= threshold
  occupied) when the snapshot is ready, otherwise all -1 with the same geometry
  (the old adapter published an all-unknown grid whenever it was not ready).
"""

from nav_msgs.msg import OccupancyGrid

PLANNING_SNAPSHOT_TOPIC = "/rog_map_adapter/planning_snapshot"


def snapshot_to_grid(snapshot):
    grid = OccupancyGrid()
    grid.header.frame_id = snapshot.header.frame_id
    grid.header.stamp = snapshot.source_stamp
    grid.info = snapshot.info
    cells = int(snapshot.info.width) * int(snapshot.info.height)
    if snapshot.ready and len(snapshot.occupancy) == cells:
        grid.data = list(snapshot.occupancy)
    else:
        grid.data = [-1] * cells
    return grid
