"""Same as slam.launch.py but for the unitree_go2_ros2_jazzy (junior_ctrl, real-walking)
robot: base_link instead of base_footprint, different laser range. Run
unitree_go2_nav_bringup.launch.py (in the separate unitree_go2_ros2_jazzy1 workspace)
first -- this only starts slam_toolbox.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(
        get_package_share_directory('go2_navigation'),
        'config', 'mapper_params_junior.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([slam])
