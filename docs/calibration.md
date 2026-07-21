# Calibration checklist

## Camera extrinsic

Measure the rigid transform from `base_link` to `camera_1_link`. Store it as
`[x, y, z]` metres and an xyzw quaternion in
`config/calibration/camera_extrinsics.yaml`. The launch deliberately refuses an
all-zero translation when `publish: true`.

Verify the complete chain:

```bash
ros2 run tf2_ros tf2_echo base_link camera_1_color_optical_frame
```

Move a tagged object to several known points and compare `/get_object_pose`
after transformation into `base_link`. Calibration error directly becomes
grasp error.

## Tag-to-object transform

The AprilTag node estimates `T_camera_tag`. The bridge composes:

```text
T_camera_object = T_camera_tag * T_tag_object
```

`T_tag_object` must locate the same origin and axes used by the object mesh and
grasp file. Measure it from CAD or a repeatable fixture; do not compensate for
an incorrect value by arbitrarily editing grasp poses.

## End-effector frame

Measure `link6 -> grasp_frame` at the physical center between the fingertips.
The seed in the ARX robot repository is `0.11 m` along `link6 +X`. Update both
the normal and cuMotion robot descriptions if the measurement changes.

## Workspace and poses

Validate the behavior-tree workspace diagonal, nvblox bounds, home pose, and
drop pose in RViz. Keep the first execution slow and clear of people and
unmodeled obstacles.
