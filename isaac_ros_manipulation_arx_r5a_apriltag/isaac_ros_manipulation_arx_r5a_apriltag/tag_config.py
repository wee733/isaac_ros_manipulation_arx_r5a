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

"""Validated AprilTag-to-object configuration loading."""

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Dict, Tuple

import yaml

from .pose_math import normalize_quaternion


@dataclass(frozen=True)
class TagObjectConfig:
    """Metadata and rigid transform for one tagged object."""

    tag_id: int
    class_id: str
    object_name: str
    mesh_file_path: str
    confidence: float
    dimensions: Tuple[float, float, float]
    tag_to_object_translation: Tuple[float, float, float]
    tag_to_object_rotation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class TagMapConfig:
    """AprilTag detector and object mapping configuration."""

    tag_family: str
    tag_size: float
    objects: Dict[int, TagObjectConfig]


def _vector(values, size: int, field_name: str) -> tuple:
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f'{field_name} must be a list of {size} numbers')
    try:
        vector = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must contain only numbers') from error
    if not all(isfinite(value) for value in vector):
        raise ValueError(f'{field_name} must contain only finite numbers')
    return vector


def load_tag_map(path: str) -> TagMapConfig:
    """Load and validate an object mapping YAML file."""
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f'AprilTag object config not found: {config_path}')

    with config_path.open('r', encoding='utf-8') as config_file:
        raw = yaml.safe_load(config_file)

    if not isinstance(raw, dict):
        raise ValueError('AprilTag object config must contain a YAML mapping')
    if raw.get('schema_version') != 1:
        raise ValueError('Only AprilTag object config schema_version 1 is supported')

    tag_family = str(raw.get('tag_family', '')).strip()
    if not tag_family:
        raise ValueError('tag_family must be a non-empty string')
    tag_size = float(raw.get('tag_size', 0.0))
    if not isfinite(tag_size) or tag_size <= 0.0:
        raise ValueError('tag_size must be greater than zero')

    raw_objects = raw.get('objects')
    if not isinstance(raw_objects, dict) or not raw_objects:
        raise ValueError('objects must contain at least one tag mapping')

    objects = {}
    for raw_tag_id, raw_object in raw_objects.items():
        try:
            tag_id = int(raw_tag_id)
        except (TypeError, ValueError) as error:
            raise ValueError(f'Invalid AprilTag ID: {raw_tag_id!r}') from error
        if tag_id < 0:
            raise ValueError(f'AprilTag ID must be non-negative, got {tag_id}')
        if not isinstance(raw_object, dict):
            raise ValueError(f'Object mapping for tag {tag_id} must be a mapping')

        class_id = str(raw_object.get('class_id', '')).strip()
        object_name = str(raw_object.get('object_name', '')).strip()
        mesh_file_path = str(raw_object.get('mesh_file_path', '')).strip()
        if not class_id or not object_name:
            raise ValueError(f'Tag {tag_id} requires class_id and object_name')

        confidence = float(raw_object.get('confidence', 1.0))
        if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError(f'Tag {tag_id} confidence must be in [0, 1]')
        dimensions = _vector(raw_object.get('dimensions'), 3, 'dimensions')
        if any(value <= 0.0 for value in dimensions):
            raise ValueError(f'Tag {tag_id} dimensions must be greater than zero')

        transform = raw_object.get('tag_to_object')
        if not isinstance(transform, dict):
            raise ValueError(f'Tag {tag_id} requires tag_to_object')
        translation = _vector(transform.get('translation'), 3, 'translation')
        rotation = normalize_quaternion(
            _vector(transform.get('rotation'), 4, 'rotation')
        )
        objects[tag_id] = TagObjectConfig(
            tag_id=tag_id,
            class_id=class_id,
            object_name=object_name,
            mesh_file_path=mesh_file_path,
            confidence=confidence,
            dimensions=dimensions,
            tag_to_object_translation=translation,
            tag_to_object_rotation=rotation,
        )

    return TagMapConfig(tag_family=tag_family, tag_size=tag_size, objects=objects)
