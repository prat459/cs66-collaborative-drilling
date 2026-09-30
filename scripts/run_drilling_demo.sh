#!/bin/bash

set -e

echo "========================================"
echo " CS66 REPEATED DRILLING DEMO"
echo "========================================"

cd ~/elite_ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

echo
echo "[1] Checking External Control..."

TASK_RUNNING=$(ros2 topic echo \
  /io_and_status_controller/robot_task_running \
  --once 2>/dev/null | grep "data:" | awk '{print $2}')

if [ "$TASK_RUNNING" != "true" ]; then

    echo "External Control is FALSE."
    echo "Attempting to restore it..."

    ros2 service call \
      /io_and_status_controller/resend_external_script \
      std_srvs/srv/Trigger "{}"

    sleep 2

    TASK_RUNNING=$(ros2 topic echo \
      /io_and_status_controller/robot_task_running \
      --once 2>/dev/null | grep "data:" | awk '{print $2}')
fi

if [ "$TASK_RUNNING" != "true" ]; then
    echo
    echo "ERROR: External Control is still FALSE."
    echo "Robot will NOT execute."
    exit 1
fi

echo "External Control: TRUE"

echo
echo "[2] Checking trajectory controller..."

CONTROLLER_STATE=$(ros2 control list_controllers | \
  grep scaled_joint_trajectory_controller || true)

echo "$CONTROLLER_STATE"

if ! echo "$CONTROLLER_STATE" | grep -q "active"; then
    echo
    echo "ERROR: scaled_joint_trajectory_controller is not active."
    echo "Robot will NOT execute."
    exit 1
fi

echo "Trajectory controller: ACTIVE"

echo
echo "[3] Starting repeated drilling demo..."
echo
echo "Current robot pose will become UP."
echo "Stroke comes from drilling_cycle.cpp."
echo "Duration = 10 seconds."
echo
echo "Press Ctrl+C to abort."
echo

sleep 2

ros2 run cs66_drilling_control drilling_cycle \
  --ros-args \
  -p execute:=true \
  -p duration:=10.0

echo
echo "========================================"
echo " DRILLING DEMO FINISHED"
echo "========================================"
