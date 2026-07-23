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

"""Tests for dependency-free AprilTag observation filtering."""

from math import cos, radians, sin, sqrt

from isaac_ros_manipulation_arx_r5a_apriltag.observation_filter import (
    axis_aligned_bbox,
    is_fresh,
    minimum_tag_edge_px,
    StablePoseFilter,
)
import pytest


def test_tag_geometry_helpers():
    """Tag corners produce a conservative axis-aligned box and edge size."""
    corners = ((10.0, 20.0), (30.0, 20.0), (30.0, 40.0), (10.0, 40.0))
    assert minimum_tag_edge_px(corners) == pytest.approx(20.0)
    assert axis_aligned_bbox(corners) == pytest.approx((10.0, 20.0, 30.0, 40.0))


def test_stable_pose_filter_requires_consecutive_frames_and_averages():
    """A pose is hidden until the configured number of stable frames arrives."""
    pose_filter = StablePoseFilter(min_stable_frames=3, max_translation_jump_m=0.02)
    identity = (0.0, 0.0, 0.0, 1.0)

    assert pose_filter.update(7, (0.0, 0.0, 0.2), identity).stable_pose is None
    assert pose_filter.update(7, (0.003, 0.0, 0.2), identity).stable_pose is None
    update = pose_filter.update(7, (0.006, 0.0, 0.2), identity)

    assert update.reset is False
    assert update.stable_pose is not None
    assert update.stable_pose.position == pytest.approx((0.003, 0.0, 0.2))
    assert update.stable_pose.orientation == pytest.approx(identity)


def test_translation_jump_resets_stability_window():
    """A large translation jump invalidates the previous stable sequence."""
    pose_filter = StablePoseFilter(min_stable_frames=2, max_translation_jump_m=0.01)
    identity = (0.0, 0.0, 0.0, 1.0)

    pose_filter.update(3, (0.0, 0.0, 0.1), identity)
    assert pose_filter.update(3, (0.001, 0.0, 0.1), identity).stable_pose is not None

    jump = pose_filter.update(3, (0.1, 0.0, 0.1), identity)
    assert jump.reset is True
    assert jump.stable_pose is None
    reacquired = pose_filter.update(3, (0.101, 0.0, 0.1), identity)
    assert reacquired.stable_pose is not None


def test_slow_translation_drift_cannot_pass_the_full_window_gate():
    """Small adjacent steps must not hide a large whole-window displacement."""
    pose_filter = StablePoseFilter(
        min_stable_frames=5,
        max_translation_jump_m=0.02,
    )
    identity = (0.0, 0.0, 0.0, 1.0)

    updates = [
        pose_filter.update(4, (position, 0.0, 0.1), identity)
        for position in (0.0, 0.019, 0.038, 0.057, 0.076)
    ]

    assert any(update.reset for update in updates)
    assert all(update.stable_pose is None for update in updates)


def test_rotation_jump_resets_stability_window():
    """A large orientation jump invalidates the previous stable sequence."""
    pose_filter = StablePoseFilter(
        min_stable_frames=2,
        max_translation_jump_m=0.01,
        max_rotation_jump_deg=5.0,
    )
    identity = (0.0, 0.0, 0.0, 1.0)
    quarter_turn = (0.0, 0.0, sqrt(0.5), sqrt(0.5))

    pose_filter.update(3, (0.0, 0.0, 0.1), identity)
    jump = pose_filter.update(3, (0.0, 0.0, 0.1), quarter_turn)
    assert jump.reset is True
    assert jump.stable_pose is None


def test_slow_rotation_drift_cannot_pass_the_full_window_gate():
    """Small adjacent turns must not hide a large whole-window rotation."""
    pose_filter = StablePoseFilter(
        min_stable_frames=5,
        max_translation_jump_m=0.01,
        max_rotation_jump_deg=5.0,
    )

    def yaw(degrees):
        half_angle = radians(degrees) / 2.0
        return (0.0, 0.0, sin(half_angle), cos(half_angle))

    updates = [
        pose_filter.update(5, (0.0, 0.0, 0.1), yaw(angle))
        for angle in (0.0, 4.0, 8.0, 12.0, 16.0)
    ]

    assert any(update.reset for update in updates)
    assert all(update.stable_pose is None for update in updates)


def test_quaternion_averaging_handles_equivalent_signs():
    """Equivalent q and -q samples do not cancel during quaternion averaging."""
    pose_filter = StablePoseFilter(min_stable_frames=2, max_translation_jump_m=0.01)
    quaternion = (0.0, 0.0, sqrt(0.5), sqrt(0.5))
    pose_filter.update(1, (0.0, 0.0, 0.0), quaternion)
    update = pose_filter.update(1, (0.0, 0.0, 0.0), tuple(-v for v in quaternion))
    assert update.stable_pose.orientation == pytest.approx(quaternion)


def test_freshness_rejects_old_and_future_stamps():
    """TTL validation accepts only non-future timestamps inside the TTL."""
    assert is_fresh(source_stamp_ns=900, now_ns=1000, ttl_ns=100)
    assert not is_fresh(source_stamp_ns=899, now_ns=1000, ttl_ns=100)
    assert not is_fresh(source_stamp_ns=1001, now_ns=1000, ttl_ns=100)


def test_freshness_can_allow_bounded_cross_process_clock_skew():
    """A configured tolerance accepts only small future timestamp skew."""
    assert is_fresh(
        source_stamp_ns=1050,
        now_ns=1000,
        ttl_ns=100,
        future_tolerance_ns=50,
    )
    assert not is_fresh(
        source_stamp_ns=1051,
        now_ns=1000,
        ttl_ns=100,
        future_tolerance_ns=50,
    )

    with pytest.raises(ValueError, match='future_tolerance_ns'):
        is_fresh(
            source_stamp_ns=1000,
            now_ns=1000,
            ttl_ns=100,
            future_tolerance_ns=-1,
        )
