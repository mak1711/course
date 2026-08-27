"""Background-spinning accumulator for objects the camera has detected.

Subscribes to /yolo/detections_3d (published by yolo_ros's YOLO-World + detect_3d_node
pipeline, when running) and keeps a deduped running list: same label within
DEDUPE_RADIUS_M of an existing entry updates it (keeping the highest-confidence
position seen) rather than creating a duplicate.

Runs its own node + background spin thread, independent of Go2NavClient's on-demand
spin pattern -- detections need to keep accumulating even when no navigation tool call
is in flight (e.g. while the agent is just looking around).

yolo_msgs lives in the separate junior workspace and is only on the ROS graph when that
workspace's demo is running (go2_simulation has no camera at all) -- the import is
optional so this server doesn't crash when it's unavailable, it just reports no
detections.
"""

import math
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

try:
    from yolo_msgs.msg import DetectionArray

    _YOLO_MSGS_AVAILABLE = True
except ImportError:
    DetectionArray = None
    _YOLO_MSGS_AVAILABLE = False

DEDUPE_RADIUS_M = 0.75


class _DetectedObject:
    __slots__ = ("label", "x", "y", "score", "count")

    def __init__(self, label: str, x: float, y: float, score: float):
        self.label = label
        self.x = x
        self.y = y
        self.score = score
        self.count = 1


class DetectionStore:
    def __init__(self, topic: str = "/yolo/detections_3d", node_name: str = "go2_mcp_detection_store"):
        self._lock = threading.Lock()
        self._objects: list[_DetectedObject] = []
        self._available = _YOLO_MSGS_AVAILABLE
        self.node = None
        self._executor = None
        self._thread = None
        if not self._available:
            return

        self.node = Node(node_name)
        self.node.create_subscription(DetectionArray, topic, self._on_detections, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_detections(self, msg) -> None:
        with self._lock:
            for det in msg.detections:
                pos = det.bbox3d.center.position
                # An unpopulated bbox3d (e.g. a 2D-only detection that never got a
                # depth match) comes through as an exact-zero position -- skip it
                # rather than storing a fake object at the map origin.
                if pos.x == 0.0 and pos.y == 0.0 and pos.z == 0.0:
                    continue
                label = det.class_name
                score = float(det.score)
                merged = False
                for obj in self._objects:
                    if obj.label == label and math.hypot(obj.x - pos.x, obj.y - pos.y) < DEDUPE_RADIUS_M:
                        if score > obj.score:
                            obj.x, obj.y, obj.score = pos.x, pos.y, score
                        obj.count += 1
                        merged = True
                        break
                if not merged:
                    self._objects.append(_DetectedObject(label, pos.x, pos.y, score))

    def list_objects(self) -> list[dict]:
        with self._lock:
            objs = list(self._objects)
        return [
            {
                "label": o.label,
                "x": round(o.x, 2),
                "y": round(o.y, 2),
                "confidence": round(o.score, 3),
                "times_seen": o.count,
            }
            for o in sorted(objs, key=lambda o: -o.count)
        ]

    def find(self, name: str) -> dict | None:
        """Case-insensitive, substring-tolerant lookup by label (e.g. "cylinder"
        should match a stored "yellow cylinder"). Returns the best (most-seen) match,
        or None."""
        name_lower = name.lower()
        matches = [
            o
            for o in self.list_objects()
            if name_lower in o["label"].lower() or o["label"].lower() in name_lower
        ]
        if not matches:
            return None
        return max(matches, key=lambda o: o["times_seen"])

    def destroy(self) -> None:
        if not self._available:
            return
        self._executor.shutdown()
        self.node.destroy_node()
