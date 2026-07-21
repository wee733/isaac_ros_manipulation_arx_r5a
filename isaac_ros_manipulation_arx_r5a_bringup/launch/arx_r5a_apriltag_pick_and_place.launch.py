# Copyright (c) 2026 wee733
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""Launch the patch-free ARX R5A AprilTag pick-and-place workflow."""

import os

from ament_index_python.packages import get_package_share_directory
from isaac_ros_manipulation_arx_r5a_apriltag.tag_config import load_tag_map
from isaac_ros_manipulation_arx_r5a_bringup.config import load_camera_calibration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'isaac_ros_manipulation_arx_r5a_bringup'
CONTAINER_NAME = 'manipulator_container'
DESCRIPTION_PACKAGE = 'isaac_ros_manipulation_arx_r5a_robot_description'


def _value(context, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _bool(context, name: str) -> bool:
    value = _value(context, name).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'Launch argument {name} must be a boolean, got {value!r}')


def _include(package_name: str, relative_path: str, arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name), relative_path
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(arguments or {}).items(),
    )


def _get_apriltag_nodes(tag_size: float, tag_family: str, context):
    image_topic = _value(context, 'color_image_topic')
    camera_info_topic = _value(context, 'color_camera_info_topic')
    detection_topic = _value(context, 'tag_detections_topic')
    width = int(_value(context, 'camera_width'))
    height = int(_value(context, 'camera_height'))

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='apriltag_rectify',
        namespace='camera_1',
        parameters=[{
            'output_width': width,
            'output_height': height,
        }],
        remappings=[
            ('image', image_topic),
            ('camera_info', camera_info_topic),
            ('image_rect', '/camera_1/apriltag/image_rect'),
            ('camera_info_rect', '/camera_1/apriltag/camera_info_rect'),
        ],
    )
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='camera_1',
        parameters=[{
            'size': tag_size,
            'max_tags': int(_value(context, 'max_tags')),
            'tile_size': int(_value(context, 'tile_size')),
            'tag_family': tag_family,
            'backends': _value(context, 'apriltag_backends'),
        }],
        remappings=[
            ('image', '/camera_1/apriltag/image_rect'),
            ('camera_info', '/camera_1/apriltag/camera_info_rect'),
            ('tag_detections', detection_topic),
        ],
    )
    return LoadComposableNodes(
        target_container=CONTAINER_NAME,
        composable_node_descriptions=[rectify_node, apriltag_node],
    )


def _get_nvblox_node(workspace_path: str):
    manipulation_share = get_package_share_directory('isaac_ros_manipulation_bringup')
    nvblox_examples_share = get_package_share_directory('nvblox_examples_bringup')
    parameters = [
        os.path.join(nvblox_examples_share, 'config', 'nvblox', 'nvblox_base.yaml'),
        os.path.join(
            manipulation_share, 'config', 'nvblox', 'nvblox_manipulator_base.yaml'
        ),
        os.path.join(
            manipulation_share,
            'config',
            'nvblox',
            'specializations',
            'nvblox_manipulator_realsense.yaml',
        ),
        workspace_path,
        {'num_cameras': 1, 'global_frame': 'base_link'},
    ]
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=parameters,
        remappings=[
            ('/camera_0/color/image', '/camera_1/color/image_raw'),
            ('/camera_0/color/camera_info', '/camera_1/color/camera_info'),
            (
                '/camera_0/depth/camera_info',
                '/camera_1/aligned_depth_to_color/camera_info',
            ),
            ('/camera_0/depth/image', '/cumotion/camera_1/world_depth'),
        ],
    )
    return LoadComposableNodes(
        target_container=CONTAINER_NAME,
        composable_node_descriptions=[nvblox_node],
    )


def launch_setup(context, *args, **kwargs):
    """Resolve configuration files and construct the complete launch graph."""
    tag_config_file = _value(context, 'tag_config_file')
    tag_map = load_tag_map(tag_config_file)
    calibration = load_camera_calibration(_value(context, 'camera_calibration_file'))
    enable_nvblox = _bool(context, 'enable_nvblox')
    use_sim_time = _bool(context, 'use_sim_time')
    description_share = get_package_share_directory(DESCRIPTION_PACKAGE)
    bringup_share = get_package_share_directory(PACKAGE_NAME)

    cumotion_urdf = os.path.join(description_share, 'urdf', 'r5a_cumotion.urdf')
    cumotion_xrdf = os.path.join(description_share, 'xrdf', 'r5a.xrdf')
    behavior_tree_config = _value(context, 'behavior_tree_config_file')
    blackboard_config = _value(context, 'blackboard_config_file')
    workspace_config = _value(context, 'nvblox_workspace_file')

    actions = [
        Node(
            package='rclcpp_components',
            executable='component_container_mt',
            name=CONTAINER_NAME,
            output='screen',
            arguments=['--ros-args', '--log-level', _value(context, 'log_level')],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]

    if _bool(context, 'start_camera'):
        actions.append(_include(
            'isaac_ros_manipulation_bringup',
            'launch/include/realsense.launch.py',
            {
                'num_cameras': '1',
                'camera_ids_config_name': _value(context, 'camera_ids_config_name'),
                'container_name': CONTAINER_NAME,
                'run_standalone': 'False',
                'enable_dnn_depth_in_realsense': 'False',
                'enable_depth': 'True',
            },
        ))

    if _bool(context, 'start_apriltag'):
        actions.append(_get_apriltag_nodes(tag_map.tag_size, tag_map.tag_family, context))

    if _bool(context, 'start_apriltag_object_server'):
        actions.append(Node(
            package='isaac_ros_manipulation_arx_r5a_apriltag',
            executable='apriltag_object_server',
            name='apriltag_object_server',
            output='screen',
            parameters=[{
                'tag_config_file': tag_config_file,
                'detections_topic': _value(context, 'tag_detections_topic'),
                'expected_camera_frame': _value(context, 'camera_optical_frame'),
                'pose_ttl_sec': float(_value(context, 'pose_ttl_sec')),
                'min_stable_frames': int(_value(context, 'min_stable_frames')),
                'min_tag_edge_px': float(_value(context, 'min_tag_edge_px')),
                'max_translation_jump_m': float(
                    _value(context, 'max_translation_jump_m')
                ),
                'max_rotation_jump_deg': float(
                    _value(context, 'max_rotation_jump_deg')
                ),
                'use_sim_time': use_sim_time,
            }],
        ))

    if _bool(context, 'start_object_selection_server'):
        actions.append(_include(
            'isaac_ros_manipulation_servers',
            'launch/object_selection_server.launch.py',
            {
                'action_name': '/get_selected_object',
                'selection_policy': _value(context, 'selection_policy'),
            },
        ))

    if _bool(context, 'start_robot'):
        actions.append(_include(
            'isaac_ros_manipulation_arx_r5a_driver_utils',
            'launch/arx_r5a_driver.launch.py',
            {
                'use_sim_time': str(use_sim_time),
                'start_vendor_driver': _value(context, 'start_vendor_driver'),
                'start_cumotion': 'False',
                'enable_cumotion_moveit_plugin': 'False',
                'start_rviz': _value(context, 'start_rviz'),
                'read_esdf_world': 'False',
                'log_level': _value(context, 'log_level'),
            },
        ))

    if _bool(context, 'start_motion_stack'):
        actions.append(_include(
            'isaac_ros_cumotion',
            'launch/isaac_ros_cumotion.launch.py',
            {
                'cumotion_action_server.xrdf_file_path': cumotion_xrdf,
                'cumotion_action_server.urdf_file_path': cumotion_urdf,
                'cumotion_action_server.tool_frame': 'link6',
                'cumotion_action_server.time_dilation_factor': _value(
                    context, 'time_dilation_factor'
                ),
                'cumotion_action_server.read_esdf_world': str(enable_nvblox),
                'cumotion_action_server.publish_cumotion_world_as_voxels': 'True',
                'cumotion_action_server.joint_states_topic': '/joint_states',
                'cumotion_action_server.add_ground_plane': 'False',
                'cumotion_action_server.override_moveit_scaling_factors': 'False',
                'cumotion_action_server.moveit_collision_objects_scene_file': '',
            },
        ))

        if enable_nvblox:
            actions.append(_include(
                'isaac_ros_cumotion_robot_segmenter',
                'launch/robot_segmenter.launch.py',
                {
                    'robot_segmenter.input_qos': 'SENSOR_DATA',
                    'robot_segmenter.output_qos': 'SENSOR_DATA',
                    'robot_segmenter.depth_image_topics': (
                        '[/camera_1/aligned_depth_to_color/image_raw]'
                    ),
                    'robot_segmenter.depth_camera_infos': (
                        '[/camera_1/aligned_depth_to_color/camera_info]'
                    ),
                    'robot_segmenter.robot_mask_publish_topics': (
                        '[/cumotion/camera_1/robot_mask]'
                    ),
                    'robot_segmenter.world_depth_publish_topics': (
                        '[/cumotion/camera_1/world_depth]'
                    ),
                    'robot_segmenter.distance_threshold': _value(
                        context, 'robot_mask_distance_threshold'
                    ),
                    'robot_segmenter.time_sync_slop': _value(
                        context, 'time_sync_slop'
                    ),
                    'robot_segmenter.joint_states_topic': '/joint_states',
                    'robot_segmenter.urdf_path': cumotion_urdf,
                    'robot_segmenter.xrdf_path': cumotion_xrdf,
                    'robot_segmenter.enable_cuda_mps': 'False',
                    'robot_segmenter.num_cameras': '1',
                    'robot_segmenter.container_name': CONTAINER_NAME,
                    'robot_segmenter.robot_base_frame': 'base_link',
                },
            ))
            actions.append(_get_nvblox_node(workspace_config))

        actions.append(_include(
            'isaac_ros_cumotion_object_attachment',
            'launch/object_attachment.launch.py',
            {
                'object_attachment.clear_esdf_on_attach': str(enable_nvblox),
                'object_attachment.esdf_reference_frame': 'base_link',
                'object_attachment.container_name': CONTAINER_NAME,
            },
        ))

    if calibration.publish:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arx_r5a_camera_static_transform',
            output='screen',
            arguments=[
                '--x', str(calibration.translation[0]),
                '--y', str(calibration.translation[1]),
                '--z', str(calibration.translation[2]),
                '--qx', str(calibration.rotation[0]),
                '--qy', str(calibration.rotation[1]),
                '--qz', str(calibration.rotation[2]),
                '--qw', str(calibration.rotation[3]),
                '--frame-id', calibration.parent_frame,
                '--child-frame-id', calibration.child_frame,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    else:
        actions.append(LogInfo(msg=(
            'Camera extrinsic publication is disabled. AprilTag detection can run, but '
            'pick-and-place requires a measured base_link -> camera_1_link transform.'
        )))

    if _bool(context, 'start_orchestrator'):
        actions.append(_include(
            'isaac_ros_manipulation_pick_and_place',
            'launch/orchestration.launch.py',
            {
                'behavior_tree_config_file': behavior_tree_config,
                'blackboard_config_file': blackboard_config,
                'print_ascii_tree': _value(context, 'print_ascii_tree'),
                'manual_mode': 'False',
                'log_level': _value(context, 'log_level'),
                'headless': _value(context, 'headless'),
                'frame_prefix': '',
            },
        ))

    actions.append(LogInfo(msg=(
        'ARX R5A AprilTag workflow loaded from ' + bringup_share +
        '. Send a MultiObjectPickAndPlace goal only after calibration and plan-only checks.'
    )))
    return actions


def generate_launch_description():
    """Return launch arguments and the assembled workflow."""
    package_share = FindPackageShare(PACKAGE_NAME)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('start_camera', default_value='True'),
        DeclareLaunchArgument('camera_ids_config_name', default_value=''),
        DeclareLaunchArgument('start_apriltag', default_value='True'),
        DeclareLaunchArgument(
            'start_apriltag_object_server',
            default_value=LaunchConfiguration('start_apriltag'),
        ),
        DeclareLaunchArgument('start_object_selection_server', default_value='True'),
        DeclareLaunchArgument('start_robot', default_value='True'),
        DeclareLaunchArgument('start_motion_stack', default_value='True'),
        DeclareLaunchArgument('start_vendor_driver', default_value='False'),
        DeclareLaunchArgument('start_rviz', default_value='True'),
        DeclareLaunchArgument('start_orchestrator', default_value='True'),
        DeclareLaunchArgument('enable_nvblox', default_value='True'),
        DeclareLaunchArgument('print_ascii_tree', default_value='False'),
        DeclareLaunchArgument('selection_policy', default_value='first'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument(
            'color_image_topic', default_value='/camera_1/color/image_raw'
        ),
        DeclareLaunchArgument(
            'color_camera_info_topic', default_value='/camera_1/color/camera_info'
        ),
        DeclareLaunchArgument('tag_detections_topic', default_value='/tag_detections'),
        DeclareLaunchArgument(
            'camera_optical_frame', default_value='camera_1_color_optical_frame'
        ),
        DeclareLaunchArgument('apriltag_backends', default_value='CUDA'),
        DeclareLaunchArgument('max_tags', default_value='16'),
        DeclareLaunchArgument('tile_size', default_value='4'),
        DeclareLaunchArgument('pose_ttl_sec', default_value='0.5'),
        DeclareLaunchArgument('min_stable_frames', default_value='5'),
        DeclareLaunchArgument('min_tag_edge_px', default_value='40.0'),
        DeclareLaunchArgument('max_translation_jump_m', default_value='0.02'),
        DeclareLaunchArgument('max_rotation_jump_deg', default_value='5.0'),
        DeclareLaunchArgument('time_dilation_factor', default_value='0.10'),
        DeclareLaunchArgument('time_sync_slop', default_value='0.10'),
        DeclareLaunchArgument('robot_mask_distance_threshold', default_value='0.05'),
        DeclareLaunchArgument(
            'tag_config_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'apriltag', 'tagged_cube.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'calibration', 'camera_extrinsics.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'behavior_tree_config_file',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'behavior_tree',
                'arx_r5a_apriltag_behavior_tree.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'blackboard_config_file',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'behavior_tree',
                'arx_r5a_apriltag_blackboard.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'nvblox_workspace_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'nvblox', 'workspace_bounds.yaml'
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
