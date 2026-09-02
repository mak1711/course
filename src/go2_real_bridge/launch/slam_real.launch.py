"""SLAM against the REAL Go2 -- for building a 2D map only, no autonomous movement.
Drive the robot yourself with the wireless controller while this runs; slam_toolbox
builds the map from real sensor data as you go.

Brings up everything unitree_ros2's own SDK bridge doesn't provide: real odometry +
TF (go2_real_bridge's odom_tf_bridge, republishing /utlidar/robot_odom -- the SDK's
own onboard, lidar-corrected odometry estimate, already a standard nav_msgs/Odometry
with frame_id=odom/child_frame_id=base_link -- since unitree_ros2's bridge itself
publishes no TF and no standard Odometry message at all), a pointcloud_to_laserscan
bridge from /utlidar/cloud_base (the SDK's own point cloud, already transformed into
base_link -- no static sensor-mount TF needed) to a 2D /scan, and slam_toolbox itself
(reusing go2_navigation's mapper_params_junior.yaml unmodified -- its frame/topic
names were already generic, only use_sim_time needs to be false here since there's no
/clock outside simulation).

This is the second version of this pipeline. The first one reconstructed odometry from
/lf/sportmodestate (raw leg/IMU state, manual quaternion remap + timestamp fix) and
fed pointcloud_to_laserscan the raw /utlidar/cloud through a hand-measured static TF.
That worked with the robot sitting still but produced a jumbled map once actually
driven around -- raw SportModeState isn't lidar-corrected, and a hand-measured static
TF accumulates rotation error every time the robot turns. Switched to the SDK's own
/utlidar/robot_odom and /utlidar/cloud_base, which don't have either problem.

Prerequisites (not started by this launch file):
  1. Physically connect the robot via Ethernet, set your IP to 192.168.123.99/24.
  2. source /home/kan/lab/course/unitree_ros2/setup.sh
  3. Confirm real topics are flowing: `ros2 topic hz /utlidar/cloud_base` and
     `ros2 topic hz /utlidar/robot_odom` should both show real data before launching.

Then: `ros2 launch go2_real_bridge slam_real.launch.py`, drive the robot around with
the wireless controller to sweep the lidar through the space, and once you're happy
with the map, save it the same way as any other slam_toolbox map:
  `ros2 run nav2_map_server map_saver_cli -f <name>`
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapper_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config", "mapper_params_junior.yaml")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    odom_tf_bridge = Node(
        package="go2_real_bridge",
        executable="odom_tf_bridge",
        name="robot_odom_bridge",
        output="screen",
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "target_frame": "base_link",
                "transform_tolerance": 0.05,
                # /utlidar/cloud_deskewed (not /utlidar/cloud_base): cloud_base gives
                # one static pose per ~65ms scan sweep, with no correction for robot
                # motion *during* that sweep -- fine stationary, but exactly the kind
                # of thing that smears/jumbles a moving robot's map. cloud_deskewed is
                # the SDK's own motion-corrected version of the same data (published
                # in "odom" frame; pointcloud_to_laserscan transforms it into
                # target_frame=base_link via TF before height-filtering, same as any
                # other source frame, so the height band below still applies in
                # base_link coordinates same as it did for cloud_base). Measured live:
                # 79.2% of cloud_base's points land in z=[-0.6,-0.05], only 1.6% in
                # [0.05,0.6] (the simulated flat-mounted Velodyne's band -- doesn't
                # apply here, the real L1's mount looks mostly forward-and-down).
                "min_height": -0.6,
                "max_height": -0.05,
                "angle_min": -3.14159265,
                "angle_max": 3.14159265,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                # range_min raised from 0.3 to 1.0: with the height band above, a
                # live scan (robot rotating in place) showed returns clustered at
                # 0.36-0.82m at nearly every angle, regardless of heading -- too
                # close and too uniform to be room walls. That's the floor: the L1's
                # steep downward mount tilt makes the beam intersect the nearby floor
                # at roughly the same short range no matter which way the robot
                # faces, and the height band alone doesn't separate "floor, close"
                # from "wall, far" since both can land in the same base_link z slice
                # depending on gait/pitch. Cutting off everything under 1m rejects
                # that floor band outright; real wall/furniture returns (confirmed
                # earlier: meaningful point counts out past 1.5-3m) are unaffected.
                "range_min": 1.0,
                "range_max": 20.0,
                "use_inf": True,
                "concurrency_level": 1,
            }
        ],
        remappings=[("cloud_in", "/utlidar/cloud_deskewed"), ("scan", "/scan")],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": mapper_params,
            # The one real difference from slam_junior.launch.py (which sets this
            # true): there's no /clock outside simulation. Leaving this true here
            # would make slam_toolbox wait forever for a clock message that never
            # comes.
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription([
        odom_tf_bridge,
        pointcloud_to_laserscan,
        slam,
    ])
