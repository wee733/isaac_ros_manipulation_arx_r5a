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

"""Tests for AprilTag object-map configuration validation."""

from isaac_ros_manipulation_arx_r5a_apriltag.tag_config import load_tag_map
import pytest


def test_load_tag_map_parses_object_metadata(tmp_path):
    """A valid mapping is parsed and its quaternion is normalized."""
    config_path = tmp_path / 'tags.yaml'
    config_path.write_text(
        """schema_version: 1
tag_family: tag36h11
tag_size: 0.04
objects:
  7:
    class_id: '3'
    object_name: soup_can
    mesh_file_path: /tmp/soup_can.obj
    confidence: 0.95
    dimensions: [0.067, 0.067, 0.101]
    tag_to_object:
      translation: [0.0, 0.0, -0.05]
      rotation: [0.0, 0.0, 0.0, 2.0]
""",
        encoding='utf-8',
    )

    config = load_tag_map(str(config_path))
    tagged_object = config.objects[7]
    assert config.tag_family == 'tag36h11'
    assert config.tag_size == pytest.approx(0.04)
    assert tagged_object.mesh_file_path == '/tmp/soup_can.obj'
    assert tagged_object.tag_to_object_rotation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_load_tag_map_rejects_non_finite_transform(tmp_path):
    """Non-finite transform components are rejected before pose composition."""
    config_path = tmp_path / 'tags.yaml'
    config_path.write_text(
        """schema_version: 1
tag_family: tag36h11
tag_size: 0.04
objects:
  7:
    class_id: '3'
    object_name: soup_can
    dimensions: [0.067, 0.067, 0.101]
    tag_to_object:
      translation: [.nan, 0.0, 0.0]
      rotation: [0.0, 0.0, 0.0, 1.0]
""",
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='finite'):
        load_tag_map(str(config_path))
