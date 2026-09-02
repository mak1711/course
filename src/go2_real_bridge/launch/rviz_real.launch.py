"""Standalone RViz launch for watching the real-Go2 mapping pipeline live.

Separate from slam_real.launch.py on purpose -- RViz is pure visualization (no
publishers that could affect the robot), so it can be started/stopped independently
without touching the mapping pipeline itself. Run both together:

  ros2 launch go2_real_bridge slam_real.launch.py
  ros2 launch go2_real_bridge rviz_real.launch.py    # in a second terminal

Shows: the growing /map, live /scan (red points, in the height-filtered band that
actually feeds slam_toolbox), the raw /utlidar/cloud_deskewed (cyan, unfiltered -- so
you can see everything the lidar is picking up, not just what passed the height/range
filters), the full TF tree, and an /odom trail so drift is visible directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("go2_real_bridge"),
        "config", "real_mapping.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([rviz])
