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

from math import sqrt

from isaac_ros_manipulation_arx_r5a_apriltag.pose_math import compose_pose
from isaac_ros_manipulation_arx_r5a_apriltag.pose_math import rotate_vector
import pytest


def test_rotate_vector_quarter_turn_about_z():
    quaternion = (0.0, 0.0, sqrt(0.5), sqrt(0.5))
    result = rotate_vector(quaternion, (1.0, 0.0, 0.0))
    assert result == pytest.approx((0.0, 1.0, 0.0), abs=1e-7)


def test_compose_pose_applies_tag_to_object_offset():
    position, orientation = compose_pose(
        (0.4, 0.1, 0.3),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, -0.025),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert position == pytest.approx((0.4, 0.1, 0.275))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
