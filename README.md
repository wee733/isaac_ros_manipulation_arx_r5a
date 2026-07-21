# Isaac ROS Manipulation for ARX R5A

[中文说明](README.zh-CN.md)

This repository is a patch-free ROS 2 overlay for running NVIDIA Isaac ROS 4.5
pick-and-place on an ARX R5A. It is intentionally separate from both NVIDIA's
`isaac_ros_manipulation` repository and the ARX cuMotion/MoveIt robot adapter.

The first maintained demo uses a fixed RealSense camera and an Isaac ROS
AprilTag attached to a 50 mm cube:

```text
RealSense rectified RGB -> Isaac ROS AprilTag -> AprilTag object server
                                                | /get_objects
                                                | /get_object_pose
                                                v
official Multi-Object Pick-and-Place py_trees -> cuMotion -> MoveIt execute

RealSense aligned depth + /joint_states -> Robot Segmenter -> nvblox ESDF
                                                              -> cuMotion
```

The NVIDIA repository remains unmodified. This overlay launches its public
components and action contracts directly, bypassing the upstream
`CoreConfig`/`RobotType` dispatch that currently supports only upstream robot
families.

## Repository roles

- `isaac_ros_manipulation_arx_r5a_apriltag`: converts
  `AprilTagDetectionArray` into the perception actions/services expected by the
  official behavior tree.
- `isaac_ros_manipulation_arx_r5a_bringup`: composes RealSense, Rectify,
  AprilTag, cuMotion, Robot Segmenter, nvblox, object attachment, the ARX driver,
  MoveIt, and the official orchestration launch.
- [`Isaac_Ros_CuMotion_ArxR5a`](https://github.com/wee733/Isaac_Ros_CuMotion_ArxR5a):
  owns the robot description, XRDF, ros2_control hardware, and MoveIt adapter.
- [`isaac_ros_manipulation`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_manipulation):
  remains the upstream workflow implementation.

## Install

Use an Isaac ROS 4.5 workspace. Clone this overlay on the host first:

```bash
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev
cd ${ISAAC_ROS_WS}/src
# Skip this command if isaac_ros_manipulation already exists.
git clone --branch release-4.5 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_manipulation.git
git clone https://github.com/wee733/isaac_ros_manipulation_arx_r5a.git
cd ${ISAAC_ROS_WS}
isaac-ros activate
```

Inside the Isaac ROS environment, import the pinned dependencies and prepare
the ARX vendor repository so colcon sees only its ROS 2 packages:

```bash
cd ${ISAAC_ROS_WS}/src
vcs import . \
  < isaac_ros_manipulation_arx_r5a/overlay_dependencies.repos
bash isaac_ros_manipulation_arx_r5a/scripts/prepare_r5_vendor_tree.sh

cd ${ISAAC_ROS_WS}
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_bringup
source install/setup.bash
bash src/isaac_ros_manipulation_arx_r5a/scripts/check_environment.sh
```

If NVIDIA `isaac_ros_manipulation` is not present, use `full_stack.repos`
instead of `overlay_dependencies.repos`. Both manifests pin the revisions used
for this release, including the ARX R5 ROS 2 driver and message packages.

## Configure before motion

1. Measure `base_link -> camera_1_link`, edit
   `config/calibration/camera_extrinsics.yaml`, and set `publish: true`.
2. Print a `tag36h11` tag with ID `0` and set its measured edge size in
   `config/apriltag/tagged_cube.yaml` (the demo default is `0.04 m`).
3. Measure `T_tag_object`. The supplied cube profile assumes the tag is centered
   on the top face and the cube center is `25 mm` along tag `-Z`.
4. Validate the robot repository's `link6 -> grasp_frame` offset and refine
   `config/grasps/tagged_cube_grasps.yaml` using Grasp Editor or plan-only tests.
5. Replace the example home/drop poses and nvblox bounds with measured workcell
   values.

## Safety and validation status

This is a community integration and the supplied poses are calibration seeds,
not safe commands for an arbitrary workcell. Keep an emergency stop available
and progress through perception-only, TF inspection, plan-only, gripper test,
low-speed single pick, and only then continuous operation. The repository has
21 passing ROS package tests plus an AprilTag-to-object action integration
test; final collision geometry, grasp offsets, and autonomous motion still
require validation on each physical cell.

## Run

Perception-only startup:

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_bringup \
  arx_r5a_apriltag_pick_and_place.launch.py \
  start_robot:=False start_motion_stack:=False start_orchestrator:=False \
  enable_nvblox:=False start_rviz:=False
```

Full stack (the workflow does not move until an action goal is sent):

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_bringup \
  arx_r5a_apriltag_pick_and_place.launch.py
```

Example single-bin request—replace the target with a validated safe pose:

```bash
ros2 action send_goal --feedback /multi_object_pick_and_place \
  isaac_ros_manipulation_interfaces/action/MultiObjectPickAndPlace \
  '{mode: 0, target_poses: {header: {frame_id: "base_link"}, poses: [{position: {x: 0.30, y: -0.15, z: 0.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}, class_ids: []}'
```

## Perception extension point

The motion stack depends only on `/get_objects`, `/get_object_pose`,
`/get_selected_object`, and the object metadata services. FoundationPose,
SAM/SAM2, or another fiducial backend can be added later without changing the
ARX driver, behavior tree, or cuMotion wiring. Launch the replacement action
provider and set `start_apriltag:=False`. This also disables the AprilTag Object
Server by default; set `start_object_selection_server:=False` as well if the
replacement owns `/get_selected_object`.

See [architecture](docs/architecture.md), [calibration](docs/calibration.md),
and [perception backends](docs/perception_backends.md).

## Official references

- [Isaac ROS 4.5 Pick and Place](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/tutorials/pick_and_place/tutorial_pick_and_place.html)
- [Bring Your Own Robot](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/tutorials/tutorial_bring_your_own_robot.html)
- [Manipulation reference architecture](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/reference_architecture.html)
- [Isaac ROS AprilTag](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html)
