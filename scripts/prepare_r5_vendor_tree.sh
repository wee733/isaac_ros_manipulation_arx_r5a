#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 wee733
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_src="$(dirname -- "${repository_root}")"
r5_root="${1:-${workspace_src}/R5}"

required_paths=(
  ROS2/R5_ws/src/ARX_R5_ros2_V7/arx_r5_controller
  ROS2/R5_ws/src/arxmsgros2/arm_control
  ROS2/R5_ws/src/arxmsgros2/arx5_arm_msg
)

for relative_path in "${required_paths[@]}"; do
  if [[ ! -d "${r5_root}/${relative_path}" ]]; then
    echo "Missing ${r5_root}/${relative_path}" >&2
    echo "Import R5 from the supplied .repos manifest first." >&2
    exit 1
  fi
done

ignored_paths=(
  ARX_VR_SDK
  ROS
  py
)

for relative_path in "${ignored_paths[@]}"; do
  touch "${r5_root}/${relative_path}/COLCON_IGNORE"
done

echo "Prepared ${r5_root} for ROS 2 colcon discovery."
echo "Enabled packages: arm_control, arx5_arm_msg, arx_r5_controller"
