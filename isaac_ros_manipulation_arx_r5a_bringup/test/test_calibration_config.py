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

from isaac_ros_manipulation_arx_r5a_bringup.config import (
    load_camera_calibration,
    validate_behavior_tree_pose_frame,
)
import pytest
import yaml


def test_template_calibration_does_not_publish():
    config_file = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'calibration'
        / 'camera_extrinsics.yaml'
    )
    calibration = load_camera_calibration(str(config_file))
    assert not calibration.publish
    assert calibration.parent_frame == 'base_link'
    assert calibration.child_frame == 'camera_1_link'


def test_behavior_tree_pose_frame_must_match_headerless_action_contract(tmp_path):
    config_file = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'behavior_tree'
        / 'arx_r5a_apriltag_behavior_tree.yaml'
    )
    assert validate_behavior_tree_pose_frame(
        str(config_file),
        'camera_1_color_optical_frame',
        '',
    ) == 'camera_1_color_optical_frame'

    with pytest.raises(ValueError, match='must be identical'):
        validate_behavior_tree_pose_frame(
            str(config_file),
            'camera_1_color_optical_frame',
            'base_link',
        )

    base_frame_config = yaml.safe_load(config_file.read_text(encoding='utf-8'))
    base_frame_config['behavior_tree_params']['multi_object_pick_and_place'][
        'pose_estimation'
    ]['camera_frame_id'] = 'base_link'
    base_frame_config_file = tmp_path / 'base_frame_behavior_tree.yaml'
    base_frame_config_file.write_text(
        yaml.safe_dump(base_frame_config),
        encoding='utf-8',
    )
    assert validate_behavior_tree_pose_frame(
        str(base_frame_config_file),
        'camera_1_color_optical_frame',
        'base_link',
    ) == 'base_link'
