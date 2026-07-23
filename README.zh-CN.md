# ARX R5A 的 Isaac ROS Manipulation 完整流程

这是一个独立、可叠加安装的 ROS 2 仓库，用于把 ARX R5A 接入 NVIDIA Isaac
ROS 4.5 的完整抓放工作流。它不会修改、复制或要求上传一份改过的
`isaac_ros_manipulation` 官方仓库。

首个可维护 demo 选用固定 RealSense 相机和贴在 50 mm 方块顶面的 Isaac ROS
AprilTag。这里的“二维码”是 `tag36h11` AprilTag，不是手机扫码使用的普通 QR
Code。

```text
RealSense RGB
  -> Isaac ROS Rectify
  -> Isaac ROS AprilTag
  -> 本仓库 AprilTag Object Server
       ├── /get_objects
       ├── /get_object_pose
       ├── add_mesh_to_object
       └── assign_name_to_object
  -> 官方 Multi-Object Pick-and-Place py_trees
  -> cuMotion approach / grasp / retract / place
  -> MoveIt ExecuteTrajectory
  -> ros2_control
  -> ARX R5A

RealSense aligned depth + /joint_states
  -> cuMotion Robot Segmenter
  -> /cumotion/camera_1/world_depth
  -> nvblox ESDF
  -> cuMotion 动态避障
```

## 为什么采用独立 overlay

官方 4.5 顶层 `workflows.launch.py` 会构造 `CoreConfig`、`RobotType` 和
`GripperType`，当前只注册了官方支持的机器人。继续往这些枚举里加入 ARX 会导致：

- 每次升级官方仓库都要重新合并补丁；
- 我们必须维护一份 fork；
- ARX、感知和行为树配置耦合在 NVIDIA 核心包中。

本仓库绕过这个封闭的顶层分发，只组合官方公开的底层 launch、组件、action 和
`py_trees`。官方仓库可保持干净，ARX 适配也能独立发布版本。

## 仓库分工

- `isaac_ros_manipulation_arx_r5a_apriltag`
  - 订阅 `/tag_detections`；
  - 对检测做连续帧、像素尺寸、整窗平移/姿态离散度和 TTL 过滤；
  - 计算 `T_camera_object = T_camera_tag × T_tag_object`；
  - 输出官方行为树需要的感知 action/service。
- `isaac_ros_manipulation_arx_r5a_bringup`
  - 启动 RealSense、Rectify、AprilTag；
  - 启动 ARX driver/MoveIt；
  - 启动 cuMotion、Robot Segmenter、nvblox 和 Object Attachment；
  - 加载 ARX 行为树和 blackboard YAML；
  - 发布实测相机外参。
- 现有
  [`Isaac_Ros_CuMotion_ArxR5a`](https://github.com/wee733/Isaac_Ros_CuMotion_ArxR5a)
  - 只维护机器人 URDF/SRDF/XRDF、`grasp_frame`、碰撞球、ros2_control 和
    MoveIt/cuMotion 基础能力。
- NVIDIA 官方 `isaac_ros_manipulation`
  - 保持原样，继续提供行为树、接口和服务器实现。

建议的工作区布局：

```text
${ISAAC_ROS_WS}/src/
├── isaac_ros_manipulation/                 # NVIDIA release-4.5，完全不改
├── isaac_ros_apriltag/                     # NVIDIA release-4.5
├── Isaac_Ros_CuMotion_ArxR5a/              # ARX 机器人基础层
└── isaac_ros_manipulation_arx_r5a/         # 本仓库
```

## 下载与构建

先在宿主机准备 Isaac ROS 4.5 工作区并克隆本 overlay：

```bash
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev
cd ${ISAAC_ROS_WS}/src
# isaac_ros_manipulation 已存在时跳过这一整条命令。
git clone --branch release-4.5 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_manipulation.git
git clone https://github.com/wee733/isaac_ros_manipulation_arx_r5a.git
cd ${ISAAC_ROS_WS}
isaac-ros activate
```

进入 Isaac ROS 环境后，导入锁定版本的依赖，并处理 R5 厂商仓库中 ROS 1/ROS 2
重名包，确保 colcon 只发现需要的三个 ROS 2 包：

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

如果 NVIDIA `isaac_ros_manipulation` 也未下载，请把上面的
`overlay_dependencies.repos` 换成 `full_stack.repos`。两个清单都固定到了本版本
验证过的 commit/tag，并包含 ARX R5 的 ROS 2 驱动和消息包。

```bash
# 仅在官方仓库也不存在时使用，代替上面的 overlay_dependencies 导入命令。
vcs import ${ISAAC_ROS_WS}/src \
  < ${ISAAC_ROS_WS}/src/isaac_ros_manipulation_arx_r5a/full_stack.repos
```

## 第一个 demo 的实体约定

默认配置在
`isaac_ros_manipulation_arx_r5a_bringup/config/apriltag/tagged_cube.yaml`：

- AprilTag family：`tag36h11`
- ID：`0`
- 实测 tag 边长：`0.04 m`
- 方块尺寸：`0.05 × 0.05 × 0.05 m`
- tag 位于方块顶面中心；
- 默认 `T_tag_object.translation = [0, 0, -0.025]`。

AprilTag 给出的是 tag 坐标系，不是物体 mesh 原点。以下三项必须使用同一个物体
坐标系，否则会出现“识别位置正确，但夹爪总是偏一截”的问题：

- `tag_to_object`；
- `tagged_cube.obj` 的原点；
- grasp YAML 的 `object_frame`。

## 上机前必须完成的标定

1. 实测固定相机的 `base_link -> camera_1_link`，写入
   `config/calibration/camera_extrinsics.yaml`，最后才把 `publish` 改成 `true`。
2. 用卡尺确认打印后的 tag 有效边长，并同步 `tag_size`。
3. 实测 tag 到物体中心的刚性外参 `T_tag_object`。
4. 确认 cuMotion 仓库中 `link6 -> grasp_frame = 0.11 m` 确实位于两指夹持中心。
5. 使用 Grasp Editor 或低速 plan-only 修正
   `config/grasps/tagged_cube_grasps.yaml`。
6. 校正恢复位姿、投放位姿和 `config/nvblox/workspace_bounds.yaml`。

## 安全与验证状态

这是社区适配项目，仓库中的位姿只是标定种子，不能直接视为任意工作站的安全指令。
实机必须准备急停，并按“感知-only → TF 检查 → plan-only → 夹爪单测 → 低速单次
抓取 → 连续运行”的顺序推进。两个包的测试套件覆盖 AprilTag 适配、launch/配置
契约和 ROS lint。碰撞几何、夹爪偏移和自动运动仍需在每套实体工作站上单独验收。

位姿适配器现在同时支持固定的眼在手外和随机械臂运动的眼在手上相机。配置
不同的输出坐标系后，它会使用图像的原始时间戳查询 exact-time TF；如果该时刻
的变换不可用就丢弃该帧，不会退回到“最新 TF”。由于 `GetObjectPose` 结果没有
Header，`output_frame`（为空时就是相机坐标系）必须与行为树配置中的
`behavior_tree_params.multi_object_pick_and_place.pose_estimation.camera_frame_id`
完全一致；通用 launch 同时启动本适配器与编排器时会检查这一契约。眼在手上应
使用相对规划世界固定的坐标系，通常就是 `base_link`。这套时间戳/坐标系策略已有
源码和 launch 契约测试，但完整的实机 D455 眼在手上流程尚未验收。

为保持参数兼容，`max_translation_jump_m` 和 `max_rotation_jump_deg` 的名字没有
改变，但现在约束的是整个 `min_stable_frames` 窗口内任意两帧的最大平移/旋转
离散度，而不只是相邻两帧；缓慢漂移也不能通过稳定性门控。

## 分阶段启动

只验证图像和 AprilTag 感知，不启动机器人与运动栈：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_bringup \
  arx_r5a_apriltag_pick_and_place.launch.py \
  start_robot:=False \
  start_motion_stack:=False \
  start_orchestrator:=False \
  enable_nvblox:=False \
  start_rviz:=False
```

检查：

```bash
ros2 topic hz /camera_1/apriltag/image_rect
ros2 topic echo /tag_detections --once

ros2 action send_goal /get_objects \
  isaac_ros_manipulation_interfaces/action/GetObjects \
  '{}'

ros2 action send_goal /get_object_pose \
  isaac_ros_manipulation_interfaces/action/GetObjectPose \
  '{object_id: 0, class_id: tagged_cube}'
```

完整启动：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_bringup \
  arx_r5a_apriltag_pick_and_place.launch.py
```

默认 `start_vendor_driver:=False`，适合厂商驱动已经在宿主机或另一终端运行的方式；
如需由本 launch 包含厂商驱动，可显式设置 `start_vendor_driver:=True`。

确认完整契约：

```bash
ros2 topic hz /joint_states
ros2 topic hz /cumotion/camera_1/world_depth
ros2 action list -t | grep -E \
  'get_objects|get_object_pose|get_selected_object|multi_object_pick_and_place|motion_plan|execute_trajectory|gripper_cmd|attach_object'
```

触发单料箱抓放前，必须把下面目标位姿换成已验证的安全可达位姿：

```bash
ros2 action send_goal --feedback /multi_object_pick_and_place \
  isaac_ros_manipulation_interfaces/action/MultiObjectPickAndPlace \
  '{mode: 0, target_poses: {header: {frame_id: "base_link"}, poses: [{position: {x: 0.30, y: -0.15, z: 0.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}, class_ids: []}'
```

推荐顺序：感知-only → TF/位姿检查 → cuMotion plan-only → 夹爪单测 → 低速单次抓放
→ 开启 nvblox → 连续任务。

### 可选的源工位发现门控

官方多物体行为树会持续发现下一件物体。对于“从源工位抓到目标工位”的流程，
AprilTag 适配器可以只把 `/get_objects` 的发现结果限制在一个包含边界的 XYZ AABB
内：

下面的例子让适配器直接返回 `base_link` 位姿。请先复制仓库自带的行为树 YAML
作为工作站专用配置，并把位姿输入坐标系同步改为：

```yaml
behavior_tree_params:
  multi_object_pick_and_place:
    pose_estimation:
      base_frame_id: base_link
      camera_frame_id: base_link
```

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_bringup \
  arx_r5a_apriltag_pick_and_place.launch.py \
  behavior_tree_config_file:=/absolute/path/to/base_frame_behavior_tree.yaml \
  output_frame:=base_link \
  source_zone_enabled:=True \
  source_zone_min_xyz:='[0.20, -0.30, -0.39]' \
  source_zone_max_xyz:='[0.40, -0.08, -0.25]'
```

边界坐标使用 `output_frame`；如果 `output_frame` 为空，则使用相机位姿坐标系。
实际启用时应选择 `base_link` 这类固定坐标系。判断点是应用 `tag_to_object` 后的
物体中心，并且三个轴都包含边界，即 `min_xyz[i] <= center[i] <= max_xyz[i]`。
`source_zone_enabled` 默认是 `False`，通用实机 launch 不会强加仿真工作站的
范围。物体被放到 AABB 之外后，后续 `/get_objects` 不再把它当成新任务；
`/get_object_pose` 不受这个门控影响，当前任务仍可按正常的位姿 TTL 和观测更新
规则查询它。

## FoundationPose 与 SAM 如何保留

这两个复杂感知后端没有被删掉，也没有塞进首个 demo。运动层只依赖以下稳定契约：

```text
/get_objects
/get_object_pose
/get_selected_object
add_mesh_to_object
assign_name_to_object
```

未来新增 FoundationPose、SAM/SAM2 或其他检测器时，只需提供相同 action/service，
然后设置 `start_apriltag:=False`；AprilTag Object Server 默认也会随之关闭。如果新
后端还自行提供 `/get_selected_object`，再设置
`start_object_selection_server:=False`。ARX driver、行为树、cuMotion、nvblox 和
抓取配置无需重写。详细说明见 [感知后端扩展](docs/perception_backends.md)。

## 官方参考

- [Isaac ROS 4.5 Pick and Place](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/tutorials/pick_and_place/tutorial_pick_and_place.html)
- [Bring Your Own Robot](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/tutorials/tutorial_bring_your_own_robot.html)
- [Manipulation Reference Architecture](https://nvidia-isaac-ros.github.io/v/release-4.5/reference_workflows/isaac_for_manipulation/reference_architecture.html)
- [Isaac ROS AprilTag](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html)
