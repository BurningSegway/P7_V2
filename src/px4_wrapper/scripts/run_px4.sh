#!/bin/bash
PX4_DIR=/home/ros_workspace/PX4-Autopilot
$PX4_DIR/build/px4_sitl_default/bin/px4 \
  $PX4_DIR/ROMFS/px4fmu_common -s etc/init.d-posix/rcS
