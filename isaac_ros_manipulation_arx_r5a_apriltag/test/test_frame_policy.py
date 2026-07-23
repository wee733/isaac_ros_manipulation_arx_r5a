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

"""Tests for camera-image and returned-pose frame separation."""

from isaac_ros_manipulation_arx_r5a_apriltag.frame_policy import resolve_pose_frame
import pytest


def test_empty_output_frame_preserves_camera_pose_contract():
    """The default policy remains backward compatible with camera-frame poses."""
    policy = resolve_pose_frame('d455_color_optical_frame', '')

    assert policy.image_frame == 'd455_color_optical_frame'
    assert policy.pose_frame == 'd455_color_optical_frame'
    assert not policy.transform_required


def test_distinct_output_frame_requires_transform_without_relabelling_image():
    """A base-frame pose must not relabel the associated 2D image detection."""
    policy = resolve_pose_frame('d455_color_optical_frame', 'base_link')

    assert policy.image_frame == 'd455_color_optical_frame'
    assert policy.pose_frame == 'base_link'
    assert policy.transform_required


def test_same_output_frame_does_not_require_identity_tf_lookup():
    """An explicitly identical frame keeps the pose unchanged."""
    policy = resolve_pose_frame(
        'zed_x_left_camera_optical_frame',
        ' zed_x_left_camera_optical_frame ',
    )

    assert policy.pose_frame == 'zed_x_left_camera_optical_frame'
    assert not policy.transform_required


def test_empty_image_frame_is_rejected():
    """A pose contract cannot be established without an image frame."""
    with pytest.raises(ValueError, match='image_frame'):
        resolve_pose_frame('  ', 'base_link')
