# Architecture

The repository is an overlay, not an upstream fork.

```text
NVIDIA repositories (unchanged)
  ├─ RealSense include + Rectify + AprilTag
  ├─ pick-and-place py_trees and interfaces
  ├─ cuMotion / Robot Segmenter / Object Attachment
  └─ nvblox
                 ^ public ROS actions, services, topics and launch APIs
                 |
This repository
  ├─ AprilTag object contract adapter
  └─ ARX-specific composition and configuration
                 |
ARX cuMotion repository
  └─ URDF / SRDF / XRDF / ros2_control / MoveIt
```

Important ownership rules:

- Robot kinematics, joint/controller names, gripper geometry, `grasp_frame`,
  collision spheres, and the XRDF `attached_object` frame belong in the ARX
  cuMotion repository.
- Camera calibration, workspace bounds, object/tag mappings, grasp assets, and
  the complete workflow launch belong in this repository.
- Behavior-tree implementation, cuMotion, nvblox, and perception algorithms
  remain upstream dependencies.

Runtime invariants:

- exactly one `cumotion/motion_plan` server;
- exactly one `move_group` serving `execute_trajectory`;
- exactly one object attachment server;
- one component container named `manipulator_container` for components whose
  upstream launch files target that name;
- `base_link`, `link6`, `grasp_frame`, and `camera_1_color_optical_frame` must be
  connected in TF before a pick goal is accepted;
- the nvblox global frame and cuMotion world frame are both `base_link`.
