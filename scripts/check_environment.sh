#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 wee733
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

packages=(
  arm_control
  arx5_arm_msg
  arx_r5_controller
  isaac_ros_image_proc
  isaac_ros_manipulation_bringup
  isaac_ros_manipulation_interfaces
  isaac_ros_manipulation_pick_and_place
  isaac_ros_manipulation_servers
  isaac_ros_apriltag
  isaac_ros_cumotion
  isaac_ros_cumotion_robot_segmenter
  isaac_ros_cumotion_object_attachment
  nvblox_ros
  nvblox_examples_bringup
  realsense2_camera
  isaac_ros_manipulation_arx_r5a_driver_utils
  isaac_ros_manipulation_arx_r5a_robot_description
  isaac_ros_manipulation_arx_r5a_ros2_control
  isaac_ros_manipulation_arx_r5a_apriltag
  isaac_ros_manipulation_arx_r5a_bringup
)

missing=0
for package in "${packages[@]}"; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "[ok] ${package}"
  else
    echo "[missing] ${package}"
    missing=1
  fi
done

exit "${missing}"
