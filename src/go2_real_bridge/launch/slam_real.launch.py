"""SLAM against the REAL Go2 -- for building a 2D map only, no autonomous movement.
Drive the robot yourself with the wireless controller while this runs; slam_toolbox
builds the map from real sensor data as you go.

Brings up everything unitree_ros2's own SDK bridge doesn't provide: real odometry +
TF (go2_real_bridge's odom_tf_bridge, from SportModeState -- unitree_ros2 publishes no
TF and no standard Odometry message at all), the static base_link -> utlidar_lidar
transform (the real Unitree L1 lidar's physical mount offset, taken from
unitree_go2_description's lidar_4D_lidar.xacro -- the same manufacturer-accurate value
used for the *simulated* L1 model, not a guess), a pointcloud_to_laserscan bridge from
the real 3D lidar cloud to a 2D /scan, and slam_toolbox itself (reusing
go2_navigation's mapper_params_junior.yaml unmodified -- its frame/topic names were
already generic, only use_sim_time needs to be false here since there's no /clock
outside simulation).

Prerequisites (not started by this launch file):
  1. Physically connect the robot via Ethernet, set your IP to 192.168.123.99/24.
  2. source /home/kan/lab/course/unitree_ros2/setup.sh
  3. Confirm real topics are flowing: `ros2 topic hz /utlidar/cloud` and
     `ros2 topic hz /lf/sportmodestate` should both show real data before launching.

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

    # Real Unitree L1 lidar's physical mount offset relative to base_link -- from
    # unitree_go2_description/urdf/lidar_4D_lidar.xacro's lidar_l1_joint, the same
    # manufacturer-accurate value used for the *simulated* L1 model (this project's
    # simulation actually uses a generic Velodyne instead, so this xacro isn't wired
    # into anything currently running, but its numbers describe the real hardware).
    # Child frame name matches the real SDK's actual published frame_id
    # ("utlidar_lidar", confirmed via unitree_ros2's own README/`ros2 topic echo`),
    # not the xacro's internal "lidar_l1_link" naming.
    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar",
        arguments=[
            "--x", "0.25", "--y", "-0.038", "--z", "-0.03",
            "--roll", "2.879", "--pitch", "0.0", "--yaw", "1.5705",
            "--frame-id", "base_link", "--child-frame-id", "utlidar_lidar",
        ],
    )

    odom_tf_bridge = Node(
        package="go2_real_bridge",
        executable="odom_tf_bridge",
        name="sportmode_odom_bridge",
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
                # NOT the same band as the simulated Velodyne (that sensor sits flat
                # on top of the robot, so obstacles show up above base_link). The
                # real L1's mount has a steep pitch (~165 deg, see the static TF
                # above) so it looks mostly forward-and-down: measured live against
                # the real robot, transforming the raw cloud into base_link showed
                # 92.6% of points landing in z=[-0.6,-0.05] and only 2.9% in the old
                # [0.05,0.6] sim band -- that's why the first real run's /scan had
                # only ~13/723 finite rays (near-empty, not a fragmentation issue,
                # that was fixed separately in unitree_ros2/setup.sh). Widened to
                # match where the real data actually is.
                "min_height": -0.6,
                "max_height": -0.05,
                "angle_min": -3.14159265,
                "angle_max": 3.14159265,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.5,
                "range_max": 20.0,
                "use_inf": True,
                "concurrency_level": 1,
            }
        ],
        remappings=[("cloud_in", "/utlidar/cloud"), ("scan", "/scan")],
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
        static_tf_lidar,
        odom_tf_bridge,
        pointcloud_to_laserscan,
        slam,
    ])
