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

"""Small dependency-free pose composition helpers using ROS xyzw quaternions."""

from math import isfinite, sqrt
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _float_tuple(values: Iterable[float], size: int, field_name: str) -> tuple:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise ValueError(f'{field_name} must contain {size} values, got {result!r}')
    if not all(isfinite(value) for value in result):
        raise ValueError(f'{field_name} must contain only finite values')
    return result


def normalize_quaternion(values: Iterable[float]) -> Quaternion:
    """Return a normalized xyzw quaternion."""
    x, y, z, w = _float_tuple(values, 4, 'quaternion')
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('quaternion norm must be non-zero')
    return x / norm, y / norm, z / norm, w / norm


def quaternion_multiply(lhs: Iterable[float], rhs: Iterable[float]) -> Quaternion:
    """Compose two xyzw quaternions as ``lhs * rhs``."""
    lx, ly, lz, lw = normalize_quaternion(lhs)
    rx, ry, rz, rw = normalize_quaternion(rhs)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_vector(quaternion: Iterable[float], vector: Iterable[float]) -> Vector3:
    """Rotate a vector by an xyzw quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = _float_tuple(vector, 3, 'vector')

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_pose(
    parent_position: Iterable[float],
    parent_orientation: Iterable[float],
    child_position: Iterable[float],
    child_orientation: Iterable[float],
) -> Tuple[Vector3, Quaternion]:
    """Compose ``parent_T_child`` and ``child_T_object`` poses."""
    px, py, pz = _float_tuple(parent_position, 3, 'parent_position')
    rotated = rotate_vector(parent_orientation, child_position)
    position = px + rotated[0], py + rotated[1], pz + rotated[2]
    orientation = quaternion_multiply(parent_orientation, child_orientation)
    return position, orientation
