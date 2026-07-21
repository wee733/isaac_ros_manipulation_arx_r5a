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

"""Validated workcell calibration configuration helpers."""

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from isaac_ros_manipulation_arx_r5a_apriltag.pose_math import normalize_quaternion

import yaml


@dataclass(frozen=True)
class CameraCalibration:
    """Static transform from the robot base to the RealSense root frame."""

    publish: bool
    parent_frame: str
    child_frame: str
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]


def _vector(values, size: int, field_name: str) -> tuple:
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f'{field_name} must contain {size} numeric values')
    try:
        return tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must contain only numeric values') from error


def load_camera_calibration(path: str) -> CameraCalibration:
    """Load a base-to-camera static transform from YAML."""
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f'Camera calibration file not found: {config_path}')
    with config_path.open('r', encoding='utf-8') as config_file:
        raw = yaml.safe_load(config_file)

    if not isinstance(raw, dict) or raw.get('schema_version') != 1:
        raise ValueError('Camera calibration must use schema_version 1')
    transform = raw.get('base_to_camera')
    if not isinstance(transform, dict):
        raise ValueError('Camera calibration requires base_to_camera')

    parent_frame = str(transform.get('parent_frame', '')).strip()
    child_frame = str(transform.get('child_frame', '')).strip()
    if not parent_frame or not child_frame:
        raise ValueError('Camera calibration requires parent_frame and child_frame')
    translation = _vector(transform.get('translation'), 3, 'translation')
    rotation = normalize_quaternion(_vector(transform.get('rotation'), 4, 'rotation'))
    publish = bool(raw.get('publish', False))

    if publish and max(abs(value) for value in translation) < 1e-9:
        raise ValueError(
            'Refusing to publish an all-zero base-to-camera translation; '
            'replace the calibration template with measured values'
        )
    return CameraCalibration(
        publish=publish,
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation=translation,
        rotation=rotation,
    )
