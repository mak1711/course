import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_simulation')
    pkg_share_parent = os.path.dirname(pkg_share)
    urdf_path = os.path.join(pkg_share, 'urdf', 'go2.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'go2_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        # `-r` starts the sim running immediately; no `-s` (that would be headless/
        # server-only, no GUI window) -- the GPU driver issue from Session 8 is fixed
        # (confirmed via nvidia-smi + hardware direct rendering), so the GUI is safe to
        # show now.
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_go2',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'go2',
            '-x', '0.0', '-y', '0.0', '-z', '0.45',
            '-allow_renaming', 'false',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'transform_tolerance': 0.02,
            'min_height': -0.35,
            'max_height': 0.4,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 20.0,
            'use_inf': True,
            'concurrency_level': 1,
        }],
        remappings=[('cloud_in', '/points'), ('scan', '/scan')],
    )

    return LaunchDescription([
        # LIBGL_ALWAYS_SOFTWARE/MESA_GL_VERSION_OVERRIDE forced-software-rendering
        # workaround removed (Session 10) -- was needed only while the NVIDIA driver was
        # broken (Session 8); hardware rendering (both the Intel iGPU and the GTX 1650 Ti)
        # is confirmed working now, and forcing software rendering here just makes the
        # GUI slower for no reason.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share_parent),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg_share_parent),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        pointcloud_to_laserscan,
    ])
