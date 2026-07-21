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

"""Tests for reproducible source dependency manifests."""

from pathlib import Path
import re

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_NAMES = ('full_stack.repos', 'overlay_dependencies.repos')


def _load_manifest(name):
    with (REPOSITORY_ROOT / name).open('r', encoding='utf-8') as manifest_file:
        return yaml.safe_load(manifest_file)['repositories']


def test_manifests_include_required_source_repositories():
    """Include perception, motion, image, nvblox, robot, and vendor sources."""
    required = {
        'isaac_ros_apriltag',
        'isaac_ros_image_pipeline',
        'isaac_ros_cumotion',
        'isaac_ros_nvblox',
        'R5',
        'Isaac_Ros_CuMotion_ArxR5a',
    }
    for manifest_name in MANIFEST_NAMES:
        assert required <= set(_load_manifest(manifest_name))
    assert 'isaac_ros_manipulation' in _load_manifest('full_stack.repos')


def test_manifest_versions_are_immutable():
    """Use exact commits or semantic release tags instead of moving branches."""
    version_pattern = re.compile(r'(?:[0-9a-f]{40}|v\d+\.\d+\.\d+)')
    for manifest_name in MANIFEST_NAMES:
        for repository in _load_manifest(manifest_name).values():
            version = str(repository['version'])
            assert version_pattern.fullmatch(version)


def test_r5_preparation_script_ignores_duplicate_vendor_trees():
    """Keep only the required R5 ROS 2 package tree visible to colcon."""
    script = (REPOSITORY_ROOT / 'scripts' / 'prepare_r5_vendor_tree.sh').read_text(
        encoding='utf-8'
    )
    required_block = script.split('required_paths=(', 1)[1].split(')', 1)[0]
    ignored_block = script.split('ignored_paths=(', 1)[1].split(')', 1)[0]
    assert 'ARX_VR_SDK' in script
    assert 'ROS2/R5_ws/src/arxmsgros2/arm_control' in required_block
    assert 'ROS2/R5_ws/src/arxmsgros2/arm_control' not in ignored_block
    assert 'Enabled packages: arm_control, arx5_arm_msg' in script
    assert 'arx5_arm_msg' in script
    assert 'arx_r5_controller' in script
