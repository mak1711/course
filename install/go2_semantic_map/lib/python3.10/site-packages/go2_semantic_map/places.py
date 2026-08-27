"""Manual semantic map: named place -> approach pose in the `map` frame.

Places are authored by hand in config/places.yaml (see that file's comment for how it
was derived from go2_simulation's test world). No automatic object detection/labeling
is done here -- that's a deliberate simplification for now.
"""

import math
import os
from dataclasses import dataclass

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class Place:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    description: str = ""


def default_places_path() -> str:
    return os.path.join(
        get_package_share_directory("go2_semantic_map"), "config", "places.yaml"
    )


def load_places(path: str | None = None) -> dict[str, Place]:
    path = path or default_places_path()
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    places: dict[str, Place] = {}
    for name, entry in (data.get("places") or {}).items():
        places[name] = Place(
            name=name,
            x=float(entry["x"]),
            y=float(entry["y"]),
            yaw=float(entry.get("yaw", 0.0)),
            description=str(entry.get("description", "")),
        )
    return places


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
