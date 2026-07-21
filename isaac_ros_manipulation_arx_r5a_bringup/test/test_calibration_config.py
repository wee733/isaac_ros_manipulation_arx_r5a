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

from pathlib import Path

from isaac_ros_manipulation_arx_r5a_bringup.config import load_camera_calibration


def test_template_calibration_does_not_publish():
    config_file = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'calibration'
        / 'camera_extrinsics.yaml'
    )
    calibration = load_camera_calibration(str(config_file))
    assert not calibration.publish
    assert calibration.parent_frame == 'base_link'
    assert calibration.child_frame == 'camera_1_link'
