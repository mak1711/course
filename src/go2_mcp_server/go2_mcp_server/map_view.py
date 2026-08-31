"""Builds a coarse ASCII rendering of the live occupancy grid map, small enough to fit
in an LLM's context, labeled with real map-frame coordinates -- so an agent can pick
sensible points to explore on its own (free space, spread across the room) instead of
following a fixed/hardcoded path or needing full autonomous frontier-exploration
infrastructure.
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

MAX_GRID_CELLS = 40  # per axis -- keeps the rendering compact enough for an LLM prompt

# map_server publishes /map once, latched (TRANSIENT_LOCAL) -- a subscriber with the
# default VOLATILE durability never receives that already-published message, which
# looked exactly like "no map at all" (confirmed: /map genuinely had a publisher and
# valid data the whole time).
_MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def get_map_ascii(node) -> dict:
    """One-shot fetch of the current /map, downsampled to a labeled ASCII grid."""
    result = {}

    def cb(msg):
        result["msg"] = msg

    sub = node.create_subscription(OccupancyGrid, "/map", cb, _MAP_QOS)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "msg" in result:
            break
    node.destroy_subscription(sub)

    if "msg" not in result:
        return {"ok": False, "error": "No /map received -- is the map server running?"}

    msg = result["msg"]
    w, h = msg.info.width, msg.info.height
    res = msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    data = msg.data

    # Crop to the region that's actually been explored (has real obstacle data), not
    # the full saved-map canvas -- map_saver pads the file much larger than what's
    # actually been seen, and rendering the whole thing let an agent pick "free"
    # exploration points far out in that padding: confirmed live, it picked a point
    # 7.9m away and burned its entire tool-call budget just waiting to arrive.
    margin_px = max(1, int(3.0 / res))  # keep some real free space around obstacles
    occ_px = [(px, py) for py in range(h) for px in range(w) if data[py * w + px] >= 65]
    if occ_px:
        min_px = max(0, min(p[0] for p in occ_px) - margin_px)
        max_px = min(w - 1, max(p[0] for p in occ_px) + margin_px)
        min_py = max(0, min(p[1] for p in occ_px) - margin_px)
        max_py = min(h - 1, max(p[1] for p in occ_px) + margin_px)
    else:
        # Nothing explored yet -- fall back to a fixed region around the map origin
        # rather than the whole canvas.
        cx, cy = int((0.0 - ox) / res), int((0.0 - oy) / res)
        min_px, max_px = max(0, cx - margin_px * 2), min(w - 1, cx + margin_px * 2)
        min_py, max_py = max(0, cy - margin_px * 2), min(h - 1, cy + margin_px * 2)
    crop_w, crop_h = max_px - min_px + 1, max_py - min_py + 1

    bin_size = max(1, max(crop_w, crop_h) // MAX_GRID_CELLS)
    grid_w = (crop_w + bin_size - 1) // bin_size
    grid_h = (crop_h + bin_size - 1) // bin_size

    grid_rows = []  # (y_coord_of_row_center, row_string), north/high-y first
    for gy in range(grid_h - 1, -1, -1):
        row_chars = []
        for gx in range(grid_w):
            occupied = free = 0
            for py in range(min_py + gy * bin_size, min(min_py + (gy + 1) * bin_size, min_py + crop_h)):
                base = py * w
                for px in range(min_px + gx * bin_size, min(min_px + (gx + 1) * bin_size, min_px + crop_w)):
                    v = data[base + px]
                    if v >= 65:
                        occupied += 1
                    elif 0 <= v < 65:
                        free += 1
            row_chars.append("#" if occupied else ("." if free else " "))
        y_coord = oy + (min_py + gy * bin_size + bin_size / 2) * res
        grid_rows.append((round(y_coord, 1), "".join(row_chars)))

    cell_size_m = round(bin_size * res, 2)
    col_ticks = [
        f"{gx}:{round(ox + (min_px + gx * bin_size + bin_size / 2) * res, 1)}"
        for gx in range(0, grid_w, max(1, grid_w // 8))
    ]

    lines = ["col_index:x_meters -> " + ", ".join(col_ticks)]
    for y_coord, row in grid_rows:
        lines.append(f"y={y_coord:>6} {row}")

    return {
        "ok": True,
        "grid": "\n".join(lines),
        "legend": "# = obstacle, . = free space, (blank) = unexplored/unknown",
        "cell_size_m": cell_size_m,
        "note": (
            "Each row is labeled with its real map-frame Y coordinate in meters. "
            "The header line gives real X coordinates for a few column indices -- "
            "for any other column c, x = nearest_labeled_x + (c - nearest_labeled_col) "
            "* cell_size_m. Pick '.' (free) cells spread across different parts of the "
            "room, away from '#' and blank cells, as targets for navigate_to_point(x, y) -- "
            "prefer areas you haven't already found objects near."
        ),
    }
