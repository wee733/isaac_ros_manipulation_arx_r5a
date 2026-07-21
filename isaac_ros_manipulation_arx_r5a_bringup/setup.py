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

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'isaac_ros_manipulation_arx_r5a_bringup'


def package_files(directory):
    """Return setuptools data_files entries while preserving subdirectories."""
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            entries.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, filename) for filename in files],
            ))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + package_files('config') + package_files('meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wee733',
    maintainer_email='wee733@users.noreply.github.com',
    description='ARX R5A AprilTag pick-and-place bringup.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
