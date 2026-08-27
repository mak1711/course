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
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
