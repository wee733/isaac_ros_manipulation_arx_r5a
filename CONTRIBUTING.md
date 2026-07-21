# Contributing

Keep this repository as an overlay. Do not copy or patch NVIDIA
`isaac_ros_manipulation` source into it; compose public launch files and
interfaces, and keep robot model changes in `Isaac_Ros_CuMotion_ArxR5a`.

Before opening a pull request, run from a sourced Isaac ROS 4.5 workspace:

```bash
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_bringup
colcon test --packages-select \
  isaac_ros_manipulation_arx_r5a_apriltag \
  isaac_ros_manipulation_arx_r5a_bringup
colcon test-result --verbose
```

Update contract tests when changing action names, frames, repository manifests,
or launch ownership. Keep physical calibration values out of defaults unless
they are documented as measured for the target workcell.

All commits must include a DCO sign-off:

```bash
git commit -s
```
