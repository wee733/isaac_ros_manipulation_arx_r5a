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

"""Dependency-free frame policy for timestamped object observations."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PoseFramePolicy:
    """Describe the image frame, returned pose frame, and TF requirement."""

    image_frame: str
    pose_frame: str
    transform_required: bool


def resolve_pose_frame(image_frame: str, output_frame: str = '') -> PoseFramePolicy:
    """Resolve an optional output frame without changing the image frame."""
    normalized_image_frame = str(image_frame).strip()
    normalized_output_frame = str(output_frame).strip()
    if not normalized_image_frame:
        raise ValueError('image_frame must be non-empty')

    pose_frame = normalized_output_frame or normalized_image_frame
    return PoseFramePolicy(
        image_frame=normalized_image_frame,
        pose_frame=pose_frame,
        transform_required=pose_frame != normalized_image_frame,
    )
