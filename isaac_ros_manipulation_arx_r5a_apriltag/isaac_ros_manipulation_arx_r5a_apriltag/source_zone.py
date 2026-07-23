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

"""Dependency-free source-zone geometry for object discovery."""

from dataclasses import dataclass
from math import isfinite
from typing import Tuple


Vector3 = Tuple[float, float, float]


def _vector3(values, field_name: str) -> Vector3:
    if not isinstance(values, (list, tuple)):
        raise ValueError(f'{field_name} must contain three numeric values')
    try:
        vector = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must contain three numeric values') from error
    if len(vector) != 3:
        raise ValueError(f'{field_name} must contain three numeric values')
    if not all(isfinite(value) for value in vector):
        raise ValueError(f'{field_name} must contain only finite values')
    return vector


@dataclass(frozen=True)
class AxisAlignedSourceZone:
    """Inclusive XYZ bounds expressed in the object server's pose frame."""

    minimum: Vector3
    maximum: Vector3

    def __post_init__(self) -> None:
        """Normalize numeric inputs and reject inverted bounds."""
        minimum = _vector3(self.minimum, 'source_zone_min_xyz')
        maximum = _vector3(self.maximum, 'source_zone_max_xyz')
        if any(lower > upper for lower, upper in zip(minimum, maximum)):
            raise ValueError(
                'source_zone_min_xyz must not exceed source_zone_max_xyz'
            )
        object.__setattr__(self, 'minimum', minimum)
        object.__setattr__(self, 'maximum', maximum)

    def contains(self, position) -> bool:
        """Return whether a position lies inside or on the AABB boundary."""
        point = _vector3(position, 'position')
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(point, self.minimum, self.maximum)
        )
