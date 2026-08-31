"""MCP server exposing Go2 navigation as tools an LLM can call.

Tools: list_places, navigate_to_place, get_navigation_status, cancel_navigation,
list_ros_topics, list_ros_nodes, echo_topic, rotate, get_map_overview,
navigate_to_point, list_detected_objects, set_detection_classes.
The LLM/agent that calls these never invents coordinates for a *named* place -- those
always come from list_places or a detected-object label. rotate() and
navigate_to_point() are the deliberate exceptions: raw motion primitives (turn by an
angle you choose, via Nav2's Spin behavior; drive to any (x, y) point you choose, using
get_map_overview() to see free space) that put the agent directly in charge of how the
robot moves -- for exploring, repositioning, or anything else -- rather than baking any
predefined "look around" routine into the tools themselves. Nav2 still does the actual
path planning and obstacle avoidance underneath both.

Run with: ros2 run go2_mcp_server go2_mcp_server
(stdio transport -- point an MCP client, e.g. Claude Desktop's config, at this command.)

Set GO2_PLACES_FILE to an absolute path to use a different semantic map than the
default (go2_semantic_map's places.yaml) -- e.g. places_junior.yaml for the
unitree_go2_ros2_jazzy real-walking robot, which runs a different world.
"""

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from mcp.server import MCPServer

from go2_mcp_server.detection_store import DetectionStore
from go2_mcp_server.map_view import get_map_ascii
from go2_mcp_server.nav2_client import Go2NavClient
from go2_semantic_map.places import load_places, yaw_to_quaternion

mcp = MCPServer("go2-nav")

_places = load_places(os.environ.get("GO2_PLACES_FILE"))
_nav: Go2NavClient | None = None
_detections: DetectionStore | None = None

# How far short of a detected object's raw (on-top-of-it) position to actually send
# Nav2 -- without this, navigating to a detected object repeats the exact
# "goal inside the obstacle" bug found and fixed for the hand-authored table/pillar_2
# place: Nav2 can never settle within tolerance of a point inside another object.
OBJECT_STANDOFF_M = 0.9


def _pose_for(place) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = place.x
    pose.pose.position.y = place.y
    qx, qy, qz, qw = yaw_to_quaternion(place.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _pose_for_detected_object(obj: dict) -> PoseStamped:
    """Approach pose for a detected object: stood off OBJECT_STANDOFF_M short of its
    raw position, along the line from the robot's current position, facing it --
    same idea as the approach poses hand-authored in places.yaml, computed instead of
    guessed since we don't know the object's real footprint."""
    ox, oy = obj["x"], obj["y"]
    current = _nav.get_current_position()
    rx, ry = current if current is not None else (0.0, 0.0)

    dx, dy = ox - rx, oy - ry
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        # Degenerate (robot already ~on top of the object's reported position) --
        # just aim at it directly rather than dividing by zero.
        tx, ty, yaw = ox, oy, 0.0
    else:
        ux, uy = dx / dist, dy / dist
        standoff = min(OBJECT_STANDOFF_M, max(0.0, dist - 0.1))
        tx, ty = ox - ux * standoff, oy - uy * standoff
        yaw = math.atan2(uy, ux)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = tx
    pose.pose.position.y = ty
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


@mcp.tool()
def list_places() -> list[dict]:
    """List every named place the robot can be sent to, with a short description of
    where it is. Call this to find the right name before calling navigate_to_place."""
    return [{"name": p.name, "description": p.description} for p in _places.values()]


@mcp.tool()
def navigate_to_place(name: str) -> dict:
    """Send the robot to a named place -- either from the semantic map (see
    list_places) or the label of something found by navigate_to_point()/
    list_detected_objects (case-insensitive, e.g. "cylinder" matches a detected
    "yellow cylinder"). Returns as soon as Nav2 accepts the goal -- it does NOT wait
    for arrival. Call get_navigation_status() afterwards to check progress or
    completion."""
    place = _places.get(name)
    if place is not None:
        pose = _pose_for(place)
    else:
        obj = _detections.find(name) if _detections is not None else None
        if obj is None:
            known = ", ".join(sorted(_places)) or "(none)"
            detected = ", ".join(sorted({o["label"] for o in _detections.list_objects()})) if _detections else ""
            return {
                "ok": False,
                "error": (
                    f"Unknown place '{name}'. Known places: {known}. "
                    f"Detected objects: {detected or '(none yet -- try navigate_to_point() and rotate() first)'}"
                ),
            }
        pose = _pose_for_detected_object(obj)
    try:
        _nav.navigate_to_pose(name, pose)
    except RuntimeError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "message": f"Navigation to '{name}' started."}


@mcp.tool()
def rotate(angle_deg: float) -> dict:
    """Turn the robot in place, right where it's standing, by the given angle in
    degrees (positive = turn left/counter-clockwise, negative = turn right/clockwise).
    No driving involved -- use this to let the camera see more of the room from the
    current spot. You choose the angle: a small increment to check
    list_detected_objects() between turns, or 360 to sweep everything at once. Blocks
    until the turn finishes."""
    try:
        result = _nav.spin(target_yaw=math.radians(angle_deg))
    except RuntimeError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": result.get("state") == "succeeded", "state": result.get("state")}


@mcp.tool()
def get_map_overview() -> dict:
    """Get a coarse ASCII rendering of the known map (obstacles / free space /
    unexplored), labeled with real map-frame coordinates. Use this to pick where to go
    with navigate_to_point(x, y) -- the map has no labels of its own, so this is how
    the agent finds where the open areas actually are."""
    return get_map_ascii(_nav.node)


@mcp.tool()
def navigate_to_point(x: float, y: float) -> dict:
    """Drive the robot to any (x, y) point you choose in the map frame -- a general
    movement primitive, not limited to a named place or a fixed purpose: use it to
    move anywhere on the map for any reason (explore, reposition, get closer to
    something, back away, etc). Use get_map_overview() first to pick a free ('.')
    point; sending a point inside an obstacle ('#') or unexplored (blank) area will
    fail or behave unpredictably. Returns as soon as Nav2 accepts the goal -- poll
    get_navigation_status() for arrival, then check list_detected_objects() /
    rotate() to see what the camera found along the way or from the new spot."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    try:
        _nav.navigate_to_pose(f"({x:.2f}, {y:.2f})", pose)
    except RuntimeError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "message": f"Navigating to ({x:.2f}, {y:.2f})."}


@mcp.tool()
def list_detected_objects() -> list[dict]:
    """List every object the camera has detected so far this run: label, approximate
    (x, y) position in the map frame, confidence, and how many times it's been seen
    (higher = more reliable), most-seen first. Empty if object detection isn't running
    or nothing has been found yet -- try navigate_to_point()/rotate() first."""
    if _detections is None:
        return []
    return _detections.list_objects()


@mcp.tool()
def get_navigation_status() -> dict:
    """Get the robot's current navigation state: idle, navigating (with distance
    remaining and elapsed time), succeeded, aborted, or canceled."""
    return _nav.get_status()


@mcp.tool()
def cancel_navigation() -> dict:
    """Cancel the current navigation goal, if the robot is navigating."""
    return {"ok": _nav.cancel()}


@mcp.tool()
def list_ros_topics() -> list[dict]:
    """List every ROS 2 topic currently visible on this robot's ROS graph, with its
    message type(s). Read-only introspection -- for answering questions like "what
    topics are available" or "is /scan publishing", not for navigation."""
    return [
        {"topic": name, "types": types}
        for name, types in sorted(_nav.node.get_topic_names_and_types())
    ]


@mcp.tool()
def list_ros_nodes() -> list[str]:
    """List every ROS 2 node currently running on this robot's ROS graph. Read-only
    introspection, for general questions like "what nodes are running" -- not for
    navigation."""
    return sorted(_nav.node.get_node_names())


@mcp.tool()
def echo_topic(topic_name: str) -> dict:
    """One-shot read of the most recent message on any ROS 2 topic, by exact name
    (e.g. "/scan" or "/odom") -- general read-only introspection for questions like
    "what's currently on topic X" or inspecting a topic list_ros_topics() found, not
    for navigation. Resolves the message type automatically. Times out after a few
    seconds if nothing is being published."""
    return _nav.echo_topic(topic_name)


@mcp.tool()
def set_detection_classes(classes: list[str]) -> dict:
    """Change what the camera's object detector is looking for, at runtime (calls
    YOLO-World's open-vocabulary /yolo/set_classes service) -- use this if asked to
    find something not covered by the current/default vocabulary. Pass a list of
    short noun phrases (e.g. ["red chair", "backpack"]); detection only recognizes
    classes currently set, and a narrower, targeted list can also improve accuracy
    for a specific search. Returns the classes actually applied. After calling this,
    detections already in list_detected_objects() from the old vocabulary are still
    there -- look around again (rotate()/navigate_to_point()) to find the new ones."""
    if _detections is None:
        return {"ok": False, "error": "Object detection isn't running."}
    return _detections.set_classes(classes)


def main() -> None:
    global _nav, _detections
    rclpy.init(args=None)
    _nav = Go2NavClient()
    _detections = DetectionStore()
    try:
        mcp.run()
    finally:
        _detections.destroy()
        _nav.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
