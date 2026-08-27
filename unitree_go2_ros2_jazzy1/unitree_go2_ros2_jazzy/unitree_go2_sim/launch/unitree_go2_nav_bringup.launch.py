"""Everything needed to drive this robot from Nav2: the base sim (on the flat
default.sdf world -- the bundled elevation.sdf can tip the robot on spawn before it's
even standing), a pointcloud_to_laserscan converting the Velodyne cloud to /scan, and
auto_stand.py to walk junior_ctrl's FSM into MOVE_BASE (the state that listens to
/cmd_vel) without needing a human at a keyboard.

Does NOT include SLAM or Nav2 themselves -- those come from go2_navigation in the
separate course/ workspace (this package doesn't depend on it, and shouldn't start
depending on it just for a launch file). Run this, then go2_navigation's
slam_junior.launch.py or nav2_junior.launch.py in another terminal.
"""

import os

import launch_ros
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    unitree_go2_sim_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim"
    ).find("unitree_go2_sim")
    unitree_go2_description_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description"
    ).find("unitree_go2_description")

    default_world_path = os.path.join(
        unitree_go2_description_share, "worlds", "default.sdf"
    )

    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_sim_share, "launch", "unitree_go2_launch.py")
        ),
        launch_arguments={
            # rviz stays off here -- nav2_junior.launch.py brings up its own RViz with
            # the map/AMCL/costmap displays this base-robot view doesn't have; running
            # both would just be two overlapping windows.
            "rviz": "false",
            "gui": "true",
            "world": default_world_path,
            "world_init_x": "0.0",
            "world_init_y": "0.0",
            "world_init_z": "0.35",
        }.items(),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_link",
                "transform_tolerance": 0.05,
                # Body/leg clearance band: the walking gait moves legs/feet through a
                # wide low band, so min_height sits above them; max_height stays well
                # under the lidar's own mount height to skip empty air above obstacles.
                "min_height": 0.05,
                "max_height": 0.6,
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
        remappings=[("cloud_in", "/velodyne_points/points"), ("scan", "/scan")],
    )

    auto_stand = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("unitree_go2_sim"), "lib", "unitree_go2_sim",
                "auto_stand.py",
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        sim_ld,
        pointcloud_to_laserscan,
        auto_stand,
    ])
