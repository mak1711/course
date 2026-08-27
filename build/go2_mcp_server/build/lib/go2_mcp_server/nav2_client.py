"""Thread-safe wrapper around Nav2's NavigateToPose action client.

Used from MCP tool handlers, which the MCP framework may invoke from worker threads.
No background spin thread is used: each call spins the node just long enough to get
the result it needs, under a lock, so two tool calls never spin the same node at once.
"""

import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node

_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "navigating",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


class Go2NavClient:
    """One instance per process; call from `main()` after `rclpy.init()`."""

    def __init__(self, node_name: str = "go2_mcp_nav_client"):
        self._lock = threading.Lock()
        self.node = Node(node_name)
        self._client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self._spin_client = ActionClient(self.node, Spin, "spin")
        self._goal_handle = None
        self._result_future = None
        self._last_feedback = None
        self._active_place = None

    def navigate_to_pose(self, place_name: str, pose: PoseStamped) -> None:
        """Send a new goal, waiting only until it is accepted (not until it finishes)."""
        with self._lock:
            if not self._client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(
                    "navigate_to_pose action server not available -- is Nav2 running?"
                )

            self._last_feedback = None
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            def feedback_cb(feedback_msg):
                self._last_feedback = feedback_msg.feedback

            send_future = self._client.send_goal_async(
                goal_msg, feedback_callback=feedback_cb
            )
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()
            if goal_handle is None:
                raise RuntimeError("Timed out waiting for navigate_to_pose to accept the goal.")
            if not goal_handle.accepted:
                raise RuntimeError("navigate_to_pose rejected the goal.")

            self._goal_handle = goal_handle
            self._active_place = place_name
            self._result_future = goal_handle.get_result_async()

    def get_status(self) -> dict:
        with self._lock:
            if self._goal_handle is None:
                return {"state": "idle"}

            # Pump the node briefly so any pending feedback/result callbacks run.
            rclpy.spin_once(self.node, timeout_sec=0.2)

            info = {"target": self._active_place}
            if self._result_future is not None and self._result_future.done():
                result = self._result_future.result()
                info["state"] = _STATUS_NAMES.get(result.status, f"status_{result.status}")
                return info

            info["state"] = "navigating"
            fb = self._last_feedback
            if fb is not None:
                info["distance_remaining_m"] = round(float(fb.distance_remaining), 3)
                info["navigation_time_sec"] = fb.navigation_time.sec
                info["number_of_recoveries"] = fb.number_of_recoveries
            return info

    def get_current_position(self) -> tuple[float, float] | None:
        """One-shot fetch of the robot's current (x, y) in the map frame, via
        /amcl_pose. Used to compute a standoff point when navigating to a detected
        object's raw (on-top-of-the-object) position -- reused across callers rather
        than each needing its own one-shot subscription."""
        with self._lock:
            result = {}

            def cb(msg):
                result["pos"] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

            sub = self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", cb, 1)
            for _ in range(15):
                rclpy.spin_once(self.node, timeout_sec=0.2)
                if "pos" in result:
                    break
            self.node.destroy_subscription(sub)
            return result.get("pos")

    def spin(self, target_yaw: float = math.tau, time_allowance_sec: float = 30.0) -> dict:
        """Blocking full turn-in-place via Nav2's Spin behavior -- lets the camera
        sweep the room without driving anywhere. Unlike navigate_to_pose (which only
        waits for goal acceptance, since a drive can take minutes), a spin is short
        enough to just block for the actual result -- the caller can check
        list_detected_objects() immediately after this returns."""
        with self._lock:
            if not self._spin_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(
                    "spin action server not available -- is Nav2's behavior_server running?"
                )

            goal_msg = Spin.Goal()
            goal_msg.target_yaw = target_yaw
            goal_msg.time_allowance = Duration(sec=int(time_allowance_sec))

            send_future = self._spin_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()
            if goal_handle is None:
                raise RuntimeError("Timed out waiting for spin to accept the goal.")
            if not goal_handle.accepted:
                raise RuntimeError("spin behavior rejected the goal.")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=time_allowance_sec + 15.0
            )
            result = result_future.result()
            if result is None:
                return {"state": "timed_out"}
            return {"state": _STATUS_NAMES.get(result.status, f"status_{result.status}")}

    def cancel(self) -> bool:
        with self._lock:
            if self._goal_handle is None:
                return False
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=5.0)
            return True

    def destroy(self) -> None:
        self.node.destroy_node()
