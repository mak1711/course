"""Bridges the real Go2's onboard SDK odometry into the odom -> base_link TF
slam_toolbox needs (the SDK bridge itself broadcasts none -- checked directly, no
tf2_ros usage anywhere in the vendored unitree_ros2 examples).

Originally this reconstructed odometry from /lf/sportmodestate (raw leg/IMU state --
position + a Unitree-ordered [w,x,y,z] quaternion, requiring manual remapping and a
manual timestamp fix, see git history). That version worked while the robot sat still
but produced a jumbled map once actually driven around: SportModeState is raw/unfused
state, not corrected against the lidar the way a proper odometry estimate would be.

/utlidar/robot_odom turned out to already be a full nav_msgs/Odometry, published by
the SDK itself with frame_id=odom/child_frame_id=base_link and a standard (x,y,z,w)
quaternion -- almost certainly the SDK's own lidar-corrected state estimate, at
~148Hz. No remapping, no guesswork: just republish it under /odom and broadcast the
matching TF.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class RobotOdomBridge(Node):
    def __init__(self):
        super().__init__("robot_odom_bridge")
        self.declare_parameter("topic", "/utlidar/robot_odom")
        topic = self.get_parameter("topic").value

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)
        self.get_logger().info(f"Bridging {topic} -> /odom + TF")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobotOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
