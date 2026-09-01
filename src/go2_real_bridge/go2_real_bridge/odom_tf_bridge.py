"""Bridges the real Go2's onboard SportModeState (position + IMU orientation) into a
standard nav_msgs/Odometry publish + odom -> base_link TF broadcast, so slam_toolbox
(which expects exactly those -- the same as it gets in simulation) works against the
real robot without modification.

Nothing upstream provides this: SportModeState is a Unitree-specific message, not a
ROS-standard odometry type, and unitree_ros2's bridge doesn't broadcast any TF at all
(checked directly -- no tf2_ros usage anywhere in the vendored SDK examples).
"""

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState


class SportModeOdomBridge(Node):
    def __init__(self):
        super().__init__("sportmode_odom_bridge")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # "lf/sportmodestate" (low-frequency, ~50Hz) is plenty for SLAM/odometry and
        # avoids flooding the TF tree the way the ~500Hz "sportmodestate" topic would.
        self.declare_parameter("topic", "lf/sportmodestate")

        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        topic = self.get_parameter("topic").value

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(SportModeState, topic, self._on_state, 10)
        self.get_logger().info(f"Bridging {topic} -> /odom + {self._odom_frame}->{self._base_frame} TF")

    def _on_state(self, msg: SportModeState) -> None:
        # Use the ROBOT's own embedded timestamp (msg.stamp, a TimeSpec -- same
        # sec/nanosec fields as builtin_interfaces/Time, direct copy) rather than
        # self.get_clock().now() (this bridge's own wall-clock, i.e. the laptop's).
        # Confirmed live: the robot's onboard clock isn't NTP-synced to the laptop
        # and was ~27 minutes behind it -- /utlidar/cloud (and therefore /scan,
        # since pointcloud_to_laserscan preserves input timestamps) is stamped using
        # the robot's clock too, since that sensor's driver also runs onboard. If
        # this bridge stamped odom->base_link TF with the laptop's clock instead,
        # every /scan message would look "older than anything in the transform
        # cache" to slam_toolbox and get silently dropped -- exactly what happened:
        # zero scans ever got processed, so no map ever formed. What matters isn't
        # matching true wall-clock time, just staying on the SAME clock basis as
        # the lidar data slam_toolbox correlates this against.
        #
        # msg.stamp is a TimeSpec (Unitree's own type), not builtin_interfaces/Time --
        # same (sec, nanosec) fields, but a different message class, so it can't be
        # assigned directly into a header.stamp field. Build the real type explicitly.
        now = Time(sec=msg.stamp.sec, nanosec=msg.stamp.nanosec)

        # Unitree's IMU quaternion array is [w, x, y, z] -- confirmed directly against
        # unitree_ros2's own example code (read_low_state.cpp explicitly labels
        # quaternion[0..3] as "qw, qx, qy, qz" in its log output), NOT the
        # geometry_msgs/Quaternion (x, y, z, w) order. Getting this backwards silently
        # rotates the whole map -- the same class of bug already hit once this project
        # (the initial-pose quaternion truncation), so this remapping is deliberate,
        # verified against source, not assumed.
        qw, qx, qy, qz = msg.imu_state.quaternion

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SportModeOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
