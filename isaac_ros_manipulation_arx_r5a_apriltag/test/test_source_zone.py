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

"""Tests for dependency-free source-zone geometry."""

from isaac_ros_manipulation_arx_r5a_apriltag.source_zone import (
    AxisAlignedSourceZone,
)
import pytest


def test_source_zone_includes_interior_and_boundary_positions():
    """Objects on an AABB boundary remain discoverable."""
    zone = AxisAlignedSourceZone((-0.1, -0.2, -0.3), (0.4, 0.5, 0.6))

    assert zone.contains((0.0, 0.0, 0.0))
    assert zone.contains((-0.1, 0.5, 0.6))


def test_source_zone_excludes_positions_outside_any_axis():
    """Crossing any one bound removes an object from discovery."""
    zone = AxisAlignedSourceZone((-0.1, -0.2, -0.3), (0.4, 0.5, 0.6))

    assert not zone.contains((-0.1001, 0.0, 0.0))
    assert not zone.contains((0.0, 0.5001, 0.0))
    assert not zone.contains((0.0, 0.0, 0.6001))


def test_source_zone_normalizes_numeric_lists():
    """ROS double-array values are normalized to immutable float tuples."""
    zone = AxisAlignedSourceZone([0, 1, 2], [3, 4, 5])

    assert zone.minimum == (0.0, 1.0, 2.0)
    assert zone.maximum == (3.0, 4.0, 5.0)


def test_source_zone_rejects_malformed_or_inverted_bounds():
    """Misconfigured discovery bounds fail before the action servers start."""
    with pytest.raises(ValueError, match='three numeric values'):
        AxisAlignedSourceZone((0.0, 0.0), (1.0, 1.0, 1.0))
    with pytest.raises(ValueError, match='finite'):
        AxisAlignedSourceZone((0.0, 0.0, float('nan')), (1.0, 1.0, 1.0))
    with pytest.raises(ValueError, match='must not exceed'):
        AxisAlignedSourceZone((0.0, 2.0, 0.0), (1.0, 1.0, 1.0))
