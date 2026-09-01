#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
# Was commented out and pointing at cyclonedds_ws/install/setup.bash, which doesn't
# exist -- this workspace was actually built with colcon run directly in unitree_ros2/,
# so the real overlay (unitree_go/unitree_api message packages) is here instead.
# Without sourcing this, ROS 2 doesn't know the Unitree message types and topics like
# /lowstate or /utlidar/cloud won't show up correctly even with the DDS network layer
# working.
source /home/kan/lab/course/unitree_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# FragmentSize added 2026-09-01: without it, large messages (the lidar's PointCloud2 is
# ~131KB/message, 4000+ points) silently failed reassembly over this real network link
# for every full DDS subscriber (ros2 topic echo, ros2 bag record, pointcloud_to_laserscan,
# everything except lightweight tools like `ros2 topic hz` that don't need the full
# payload) -- confirmed directly: 0 messages received without this, full ~4100-point
# 360-degree clouds received cleanly with it. This is almost certainly why the first
# real-hardware mapping attempt produced a near-empty scan and a drifting/jumbled map --
# pointcloud_to_laserscan was very likely getting the same corrupted/incomplete clouds
# echo was.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces>
                        <FragmentSize>4000B</FragmentSize>
                        </General></Domain></CycloneDDS>'
