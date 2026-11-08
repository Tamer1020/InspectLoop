#!/usr/bin/env bash
# Stop InspectLoop processes left behind after Ctrl-C or a crash.
# The gz-tools ruby wrapper can orphan the `gz sim` server; this matches ONLY
# command lines that reference InspectLoop install paths, so other ROS / Gazebo
# work on the machine is not touched.
set -u
pkill -f "inspectloop_gazebo/worlds/warehouse.sdf" && echo "stopped gz sim (InspectLoop world)"
pkill -f "ros_gz_bridge/parameter_bridge.*__node:=gz_(lidar_|camera_)?bridge( |$)" && echo "stopped InspectLoop ros_gz bridges"
pkill -f "install/inspectloop_(safety|vision|mission)/lib" && echo "stopped InspectLoop nodes"
echo "cleanup done"
