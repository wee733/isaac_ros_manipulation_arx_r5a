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
    assert "if _bool(context, 'start_apriltag_object_server'):" in launch_source
    assert "default_value=LaunchConfiguration('start_apriltag')" in launch_source
    assert "if _bool(context, 'start_object_selection_server'):" in launch_source
