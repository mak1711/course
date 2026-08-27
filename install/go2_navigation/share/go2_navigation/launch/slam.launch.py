import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(
        get_package_share_directory('go2_navigation'),
        'config', 'mapper_params_online_async.yaml')

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
