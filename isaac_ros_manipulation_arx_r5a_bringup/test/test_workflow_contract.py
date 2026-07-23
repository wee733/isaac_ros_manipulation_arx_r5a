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

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load(relative_path):
    with (PACKAGE_ROOT / relative_path).open('r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def test_tag_class_matches_blackboard_object():
    tags = _load('config/apriltag/tagged_cube.yaml')
    blackboard = _load(
        'config/behavior_tree/arx_r5a_apriltag_blackboard.yaml'
    )
    tag_class = tags['objects'][0]['class_id']
    supported = blackboard['blackboard_params']['supported_objects']
    assert supported['tagged_cube']['class_id'] == tag_class


def test_behavior_tree_uses_arx_actions_and_frames():
    config = _load(
        'config/behavior_tree/arx_r5a_apriltag_behavior_tree.yaml'
    )['behavior_tree_params']['multi_object_pick_and_place']
    assert config['detect_object']['action_server_name'] == '/get_objects'
    assert config['pose_estimation']['action_server_name'] == '/get_object_pose'
    assert config['plan_to_grasp']['action_server_name'] == 'cumotion/motion_plan'
    assert config['execute_trajectory']['action_server_name'] == 'execute_trajectory'
    assert config['attach_object']['gripper_frame'] == 'link6'
    assert config['attach_object']['grasp_frame'] == 'grasp_frame'
    assert config['open_gripper']['open_position'] == 0.044
    assert config['close_gripper']['close_position'] == 0.0


def test_nvblox_workspace_uses_base_link_sized_bounds():
    config = _load('config/nvblox/workspace_bounds.yaml')
    params = config['/**']['ros__parameters']['static_mapper']
    assert params['workspace_bounds_type'] == 'bounding_box'
    assert params['workspace_bounds_min_corner_x_m'] < 0.0
    assert params['workspace_bounds_max_corner_x_m'] > 0.0


def test_launch_does_not_use_upstream_robot_dispatch():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'arx_r5a_apriltag_pick_and_place.launch.py'
    ).read_text(encoding='utf-8')
    forbidden = (
        'CoreConfig',
        'SensorConfig',
        'RobotType',
        'GripperType',
        'workflows.launch.py',
        'sensors/cameras.launch.py',
    )
    for symbol in forbidden:
        assert symbol not in launch_source


def test_perception_servers_can_be_replaced_without_name_conflicts():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'arx_r5a_apriltag_pick_and_place.launch.py'
    ).read_text(encoding='utf-8')
    assert 'if start_apriltag_object_server:' in launch_source
    assert "default_value=LaunchConfiguration('start_apriltag')" in launch_source
    assert "if _bool(context, 'start_object_selection_server'):" in launch_source


def test_apriltag_pose_frame_and_timestamp_policy_are_launch_configurable():
    """Eye-in-hand must expose exact-time TF and clock-skew controls."""
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'arx_r5a_apriltag_pick_and_place.launch.py'
    ).read_text(encoding='utf-8')

    assert "'output_frame': _value(context, 'output_frame')" in launch_source
    assert "_value(context, 'future_tolerance_sec')" in launch_source
    assert "_value(context, 'transform_timeout_sec')" in launch_source
    assert 'validate_behavior_tree_pose_frame(' in launch_source
    assert 'if start_apriltag_object_server and start_orchestrator:' in launch_source


def test_apriltag_object_server_waits_for_exact_time_tf_asynchronously():
    """A wrist image may arrive before its matching robot-state TF sample."""
    server_source = (
        PACKAGE_ROOT.parent
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'object_server.py'
    ).read_text(encoding='utf-8')

    assert 'wait_for_transform_async(' in server_source
    assert 'Time.from_msg(message.header.stamp)' in server_source
    assert '_pending_transform' in server_source
    assert 'ClockType.STEADY_TIME' in server_source
    assert 'clock=self._transform_timeout_clock' in server_source
    assert 'lookup_transform(' not in server_source


def test_apriltag_object_server_preserves_each_detection_pose_frame():
    """2D hypotheses stay in the image frame while 3D poses use output_frame."""
    server_source = (
        PACKAGE_ROOT.parent
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'object_server.py'
    ).read_text(encoding='utf-8')

    detection_2d_body = server_source.split(
        '        detection_2d = Detection2D()', maxsplit=1
    )[1].split('        detection_3d = Detection3D()', maxsplit=1)[0]
    detection_3d_body = server_source.split(
        '        detection_3d = Detection3D()', maxsplit=1
    )[1].split('        object_info = ObjectInfo()', maxsplit=1)[0]

    assert 'cached.image_position' in detection_2d_body
    assert 'cached.image_orientation' in detection_2d_body
    assert 'cached.position,' not in detection_2d_body
    assert 'cached.orientation,' not in detection_2d_body
    assert 'cached.position' in detection_3d_body
    assert 'cached.orientation' in detection_3d_body


def test_apriltag_adapter_declares_python_tf_dependency():
    """The tf2_ros Python module is distributed by the tf2_ros_py package."""
    manifest = (
        PACKAGE_ROOT.parent
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'package.xml'
    ).read_text(encoding='utf-8')

    assert '<exec_depend>tf2_ros_py</exec_depend>' in manifest


def test_source_zone_gate_is_optional_and_only_filters_discovery():
    """A placed object must not start a new BT cycle but remains pose-queryable."""
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'arx_r5a_apriltag_pick_and_place.launch.py'
    ).read_text(encoding='utf-8')
    server_source = (
        PACKAGE_ROOT.parent
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'isaac_ros_manipulation_arx_r5a_apriltag'
        / 'object_server.py'
    ).read_text(encoding='utf-8')

    assert "'source_zone_enabled': _bool(context, 'source_zone_enabled')" in (
        launch_source
    )
    assert "'source_zone_min_xyz': _xyz(context, 'source_zone_min_xyz')" in (
        launch_source
    )
    assert "'source_zone_max_xyz': _xyz(context, 'source_zone_max_xyz')" in (
        launch_source
    )
    assert "'source_zone_enabled',\n            default_value='False'" in launch_source

    get_objects_body = server_source.split(
        '    def _execute_get_objects', maxsplit=1
    )[1].split('    def _execute_get_object_pose', maxsplit=1)[0]
    get_pose_body = server_source.split(
        '    def _execute_get_object_pose', maxsplit=1
    )[1].split('    def _add_mesh_to_object', maxsplit=1)[0]
    assert 'self._source_zone.contains(cached.position)' in get_objects_body
    assert '_source_zone' not in get_pose_body
