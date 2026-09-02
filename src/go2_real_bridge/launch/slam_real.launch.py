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
                # /utlidar/cloud_base, NOT /utlidar/cloud_deskewed. cloud_deskewed was
                # tried (motion-corrected for the ~65ms scan integration window, in
                # case that explained the map jumbling while driving) but turned out
                # to have a serious bug: measured live, 88.4% of every cloud_deskewed
                # message (10000 of ~11300 points) is literal (0,0,0) padding -- not
                # marked NaN, so nothing filters it out. Since the message's frame_id
                # is "odom", pointcloud_to_laserscan transforms those (0,0,0) points
                # from odom into base_link using the current TF -- which places every
                # single one of them at "current position relative to the odom
                # origin," i.e. pointing back at wherever the robot started, from
                # wherever it currently is. As the robot moves to different poses,
                # that produces a spike/ray converging on the same fixed point from
                # every direction -- this was the actual cause of the persistent
                # radiating-starburst pattern in the map, confirmed by cross-checking
                # cloud_deskewed's points (transformed into base_link) against
                # cloud_base's natively-reported points at the same instant, with the
                # robot stationary: cloud_base's real point count (~1050-1600) closely
                # matches cloud_deskewed's non-zero fraction, and cloud_base has zero
                # degenerate (0,0,0) points. Went back to cloud_base; the motion-
                # smearing concern that motivated trying cloud_deskewed was never
                # actually confirmed as a real problem, unlike this padding bug.
                # Measured live: 79.2% of cloud_base's points land in z=[-0.6,-0.05],
                # only 1.6% in [0.05,0.6] (the simulated flat-mounted Velodyne's band
                # -- doesn't apply here, the real L1's mount looks mostly
                # forward-and-down).
                "min_height": -0.6,
                "max_height": -0.05,
                "angle_min": -3.14159265,
                "angle_max": 3.14159265,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                # range_min: user-set to 0.4m, matching the Go2's own physical
                # footprint dimension (points closer than the robot's own body can't
                # be real environment). Previously 1.0m (raised from 0.3m to reject
                # floor-plane returns). A live range histogram of in-band points
                # (0.3s bins out to 3m) showed an enormous, dominant cluster at
                # 0.3-0.9m -- 60% of ALL points, 127k sampled -- almost certainly
                # self-hits off the robot's own legs (wide angular spread across
                # nearly the whole visible FOV, not a single directional floor
                # pattern; tightly clustered height -0.2 to -0.35m). Real
                # environment returns look like the much flatter, lower-count tail
                # from ~1.0m to 2.5m. 0.4m matches the robot's static footprint, but
                # note: the self-hit cluster's live-measured extent (0.3-0.9m) is
                # WIDER than that -- a leg mid-stride reaches further than the
                # robot's resting body dimension -- so 0.4m is unlikely to fully
                # exclude leg contamination. If that shows up in the map, the real
                # fix is a proper footprint-based self-filter (exclude points by
                # robot-relative x/y position, not just range), not a bigger range
                # cutoff, since this project also needs to detect genuinely close
                # walls in a narrow corridor.
                "range_min": 0.4,
                "range_max": 20.0,
                "use_inf": True,
                "concurrency_level": 1,
            }
        ],
        remappings=[("cloud_in", "/utlidar/cloud_base"), ("scan", "/scan")],
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
