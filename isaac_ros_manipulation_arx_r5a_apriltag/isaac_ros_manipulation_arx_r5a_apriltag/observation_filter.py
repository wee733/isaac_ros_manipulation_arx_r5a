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

"""Dependency-free filtering helpers for AprilTag object observations."""

from collections import deque
from dataclasses import dataclass
from math import acos, degrees, hypot, isfinite, sqrt
from typing import Deque, Dict, Iterable, Optional, Sequence, Tuple

from .pose_math import normalize_quaternion


Point2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseEstimate:
    """A dependency-free position and orientation pair."""

    position: Vector3
    orientation: Quaternion


@dataclass(frozen=True)
class FilterUpdate:
    """Result of adding one observation to a stable-pose filter."""

    stable_pose: Optional[PoseEstimate]
    reset: bool


def _point2_tuple(point: Iterable[float]) -> Point2:
    values = tuple(float(value) for value in point)
    if len(values) != 2:
        raise ValueError(f'2D point must contain two values, got {values!r}')
    if not all(isfinite(value) for value in values):
        raise ValueError('2D point must contain only finite values')
    return values[0], values[1]


def _vector3_tuple(vector: Iterable[float]) -> Vector3:
    values = tuple(float(value) for value in vector)
    if len(values) != 3:
        raise ValueError(f'3D vector must contain three values, got {values!r}')
    if not all(isfinite(value) for value in values):
        raise ValueError('3D vector must contain only finite values')
    return values[0], values[1], values[2]


def tag_edge_lengths(corners: Sequence[Iterable[float]]) -> Tuple[float, ...]:
    """Return the four AprilTag edge lengths in pixels."""
    points = tuple(_point2_tuple(corner) for corner in corners)
    if len(points) != 4:
        raise ValueError(f'AprilTag corners must contain four points, got {len(points)}')
    return tuple(
        hypot(
            points[(index + 1) % 4][0] - points[index][0],
            points[(index + 1) % 4][1] - points[index][1],
        )
        for index in range(4)
    )


def minimum_tag_edge_px(corners: Sequence[Iterable[float]]) -> float:
    """Return the shortest AprilTag edge length in pixels."""
    return min(tag_edge_lengths(corners))


def axis_aligned_bbox(
    corners: Sequence[Iterable[float]],
) -> Tuple[float, float, float, float]:
    """Return ``xmin, ymin, xmax, ymax`` for four image-space corners."""
    points = tuple(_point2_tuple(corner) for corner in corners)
    if len(points) != 4:
        raise ValueError(f'AprilTag corners must contain four points, got {len(points)}')
    x_values = tuple(point[0] for point in points)
    y_values = tuple(point[1] for point in points)
    return min(x_values), min(y_values), max(x_values), max(y_values)


def translation_distance(lhs: Iterable[float], rhs: Iterable[float]) -> float:
    """Return Euclidean distance between two 3D translations."""
    left = _vector3_tuple(lhs)
    right = _vector3_tuple(rhs)
    return sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def rotation_distance_deg(lhs: Iterable[float], rhs: Iterable[float]) -> float:
    """Return the shortest angular distance between two quaternions in degrees."""
    left = normalize_quaternion(lhs)
    right = normalize_quaternion(rhs)
    dot_product = abs(sum(left[index] * right[index] for index in range(4)))
    return degrees(2.0 * acos(min(1.0, dot_product)))


def is_fresh(
    source_stamp_ns: int,
    now_ns: int,
    ttl_ns: int,
    future_tolerance_ns: int = 0,
) -> bool:
    """Return whether a timestamp is inside the configured age window."""
    if ttl_ns < 0:
        raise ValueError('ttl_ns must be non-negative')
    if future_tolerance_ns < 0:
        raise ValueError('future_tolerance_ns must be non-negative')
    age_ns = int(now_ns) - int(source_stamp_ns)
    return -int(future_tolerance_ns) <= age_ns <= int(ttl_ns)


def _mean_pose(samples: Sequence[PoseEstimate]) -> PoseEstimate:
    count = len(samples)
    position = tuple(
        sum(sample.position[index] for sample in samples) / count
        for index in range(3)
    )

    reference = samples[0].orientation
    quaternion_sum = [0.0, 0.0, 0.0, 0.0]
    for sample in samples:
        orientation = sample.orientation
        dot_product = sum(reference[index] * orientation[index] for index in range(4))
        sign = -1.0 if dot_product < 0.0 else 1.0
        for index in range(4):
            quaternion_sum[index] += sign * orientation[index]

    orientation = normalize_quaternion(quaternion_sum)
    return PoseEstimate(position=position, orientation=orientation)


class StablePoseFilter:
    """Require a compact window of observations before exposing a pose."""

    def __init__(
        self,
        min_stable_frames: int,
        max_translation_jump_m: float,
        max_rotation_jump_deg: float = 180.0,
    ):
        if min_stable_frames < 1:
            raise ValueError('min_stable_frames must be at least one')
        if max_translation_jump_m <= 0.0:
            raise ValueError('max_translation_jump_m must be greater than zero')
        if not isfinite(max_rotation_jump_deg) or not 0.0 < max_rotation_jump_deg <= 180.0:
            raise ValueError('max_rotation_jump_deg must be in (0, 180]')
        self._min_stable_frames = min_stable_frames
        self._max_translation_jump_m = max_translation_jump_m
        self._max_rotation_jump_deg = max_rotation_jump_deg
        self._samples: Dict[int, Deque[PoseEstimate]] = {}

    def update(
        self,
        tag_id: int,
        position: Iterable[float],
        orientation: Iterable[float],
    ) -> FilterUpdate:
        """Add an observation and return an averaged pose once it is stable."""
        sample = PoseEstimate(
            position=_vector3_tuple(position),
            orientation=normalize_quaternion(orientation),
        )
        history = self._samples.setdefault(
            int(tag_id), deque(maxlen=self._min_stable_frames)
        )

        reset = False
        history.append(sample)
        window = tuple(history)
        translation_spread = max(
            (
                translation_distance(left.position, right.position)
                for index, left in enumerate(window)
                for right in window[index + 1:]
            ),
            default=0.0,
        )
        rotation_spread = max(
            (
                rotation_distance_deg(left.orientation, right.orientation)
                for index, left in enumerate(window)
                for right in window[index + 1:]
            ),
            default=0.0,
        )
        if (
            translation_spread > self._max_translation_jump_m or
            rotation_spread > self._max_rotation_jump_deg
        ):
            history.clear()
            history.append(sample)
            reset = True

        if len(history) < self._min_stable_frames:
            return FilterUpdate(stable_pose=None, reset=reset)
        return FilterUpdate(stable_pose=_mean_pose(tuple(history)), reset=reset)

    def reset(self, tag_id: int) -> None:
        """Discard accumulated observations for one tag."""
        self._samples.pop(int(tag_id), None)

    def clear(self) -> None:
        """Discard accumulated observations for every tag."""
        self._samples.clear()
