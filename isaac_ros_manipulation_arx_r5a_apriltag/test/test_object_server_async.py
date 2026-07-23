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

"""Executable tests for the timestamped AprilTag object-server pipeline."""

from math import sqrt
from threading import RLock
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
geometry_messages = pytest.importorskip('geometry_msgs.msg')
apriltag_messages = pytest.importorskip('isaac_ros_apriltag_interfaces.msg')
pytest.importorskip('isaac_ros_manipulation_interfaces.msg')
object_server = pytest.importorskip(
    'isaac_ros_manipulation_arx_r5a_apriltag.object_server'
)
frame_policy = pytest.importorskip(
    'isaac_ros_manipulation_arx_r5a_apriltag.frame_policy'
)
observation_filter = pytest.importorskip(
    'isaac_ros_manipulation_arx_r5a_apriltag.observation_filter'
)
tag_config = pytest.importorskip(
    'isaac_ros_manipulation_arx_r5a_apriltag.tag_config'
)

TransformStamped = geometry_messages.TransformStamped
AprilTagDetection = apriltag_messages.AprilTagDetection
AprilTagDetectionArray = apriltag_messages.AprilTagDetectionArray
AprilTagObjectServer = object_server.AprilTagObjectServer
ObjectMetadata = object_server.ObjectMetadata
resolve_pose_frame = frame_policy.resolve_pose_frame
StablePoseFilter = observation_filter.StablePoseFilter
TagMapConfig = tag_config.TagMapConfig
TagObjectConfig = tag_config.TagObjectConfig


class FakeFuture:
    """Small synchronous stand-in for the tf2 ROS future."""

    def __init__(self):
        self._callbacks = []
        self._cancelled = False
        self._complete = False
        self._result = None
        self.cancel_observer = None

    def add_done_callback(self, callback):
        if self._complete or self._cancelled:
            callback(self)
        else:
            self._callbacks.append(callback)

    def cancelled(self):
        return self._cancelled

    def result(self):
        return self._result

    def cancel(self):
        self._cancelled = True
        if self.cancel_observer is not None:
            self.cancel_observer()
        self._invoke_callbacks()

    def complete(self, result):
        self._result = result
        self._complete = True
        self._invoke_callbacks()

    def _invoke_callbacks(self):
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeBuffer:
    """Record exact-time TF requests and return predetermined futures."""

    def __init__(self, futures):
        self._futures = list(futures)
        self.requests = []

    def wait_for_transform_async(self, target_frame, source_frame, time):
        self.requests.append((target_frame, source_frame, time))
        return self._futures.pop(0)


class FakeLogger:
    """Capture warnings without constructing an rclpy Node."""

    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeClock:
    """Return one deterministic ROS-time nanosecond value."""

    def __init__(self, now_ns):
        self._now_ns = now_ns

    def now(self):
        return SimpleNamespace(nanoseconds=self._now_ns)


def _detection_array(stamp_ns, position_x=0.2):
    message = AprilTagDetectionArray()
    message.header.frame_id = 'camera_optical_frame'
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000

    detection = AprilTagDetection()
    detection.family = 'tag36h11'
    detection.id = 0
    detection.pose.pose.pose.position.x = position_x
    detection.pose.pose.pose.position.z = 0.3
    detection.pose.pose.pose.orientation.w = 1.0
    for index, (x, y) in enumerate(((0, 0), (50, 0), (50, 50), (0, 50))):
        detection.corners[index].x = float(x)
        detection.corners[index].y = float(y)
    message.detections.append(detection)
    return message


def _async_server(futures):
    server = AprilTagObjectServer.__new__(AprilTagObjectServer)
    server._lock = RLock()
    server._frame_policy = resolve_pose_frame(
        'camera_optical_frame',
        'base_link',
    )
    server._expected_camera_frame = 'camera_optical_frame'
    server._tag_map = SimpleNamespace(
        tag_family='tag36h11',
        objects={0: object()},
    )
    server._tf_buffer = FakeBuffer(futures)
    server._queued_detection = None
    server._pending_transform = None
    server._pending_transform_started = None
    server._transform_timeout_sec = 0.5
    server._test_logger = FakeLogger()
    server.get_logger = lambda: server._test_logger
    server._processed = []
    server._process_detections = (
        lambda message, transform: server._processed.append((message, transform))
    )
    return server


def _transform(stamp_ns):
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_optical_frame'
    transform.header.stamp.sec = stamp_ns // 1_000_000_000
    transform.header.stamp.nanosec = stamp_ns % 1_000_000_000
    transform.transform.rotation.w = 1.0
    return transform


def test_pending_transform_keeps_newest_and_chains_exact_time_request(monkeypatch):
    """A completed request processes its image then starts only the newest one."""
    wall_time = [10.0]
    monkeypatch.setattr(object_server, 'monotonic', lambda: wall_time[0])
    first_future = FakeFuture()
    newest_future = FakeFuture()
    server = _async_server((first_future, newest_future))
    first = _detection_array(1_000_000_001, position_x=0.1)
    superseded = _detection_array(2_000_000_002, position_x=0.2)
    newest = _detection_array(3_000_000_003, position_x=0.3)

    server._on_detections(first)
    server._on_detections(superseded)
    server._on_detections(newest)

    assert server._pending_transform == (first_future, first)
    assert server._queued_detection is newest
    assert len(server._tf_buffer.requests) == 1
    first_request = server._tf_buffer.requests[0]
    assert first_request[0:2] == ('base_link', 'camera_optical_frame')
    assert first_request[2].nanoseconds == 1_000_000_001

    first_transform = _transform(1_000_000_001)
    first_future.complete(first_transform)

    assert first_transform.header.stamp == first.header.stamp
    assert server._processed == [(first, first_transform)]
    assert server._pending_transform == (newest_future, newest)
    assert server._queued_detection is None
    assert len(server._tf_buffer.requests) == 2
    newest_request = server._tf_buffer.requests[1]
    assert newest_request[0:2] == ('base_link', 'camera_optical_frame')
    assert newest_request[2].nanoseconds == 3_000_000_003
    assert all(message is not superseded for message, _ in server._processed)


def test_wall_timeout_cancels_pending_and_starts_newest(monkeypatch):
    """The steady wall-time deadline drops one request without stalling the queue."""
    wall_time = [20.0]
    monkeypatch.setattr(object_server, 'monotonic', lambda: wall_time[0])
    expired_future = FakeFuture()
    newest_future = FakeFuture()
    server = _async_server((expired_future, newest_future))
    expired = _detection_array(4_000_000_004)
    newest = _detection_array(5_000_000_005)
    pending_at_cancel = []
    expired_future.cancel_observer = (
        lambda: pending_at_cancel.append(server._pending_transform)
    )

    server._on_detections(expired)
    server._on_detections(newest)
    wall_time[0] += 0.6
    server._expire_transform_request()

    assert expired_future.cancelled()
    assert pending_at_cancel == [None]
    assert server._processed == []
    assert server._pending_transform == (newest_future, newest)
    assert server._queued_detection is None
    assert server._tf_buffer.requests[1][2].nanoseconds == 5_000_000_005
    assert len(server._test_logger.warnings) == 1
    assert 'Timed out waiting for exact-time AprilTag TF' in (
        server._test_logger.warnings[0]
    )


def test_composed_object_info_keeps_2d_and_3d_pose_frames_separate():
    """Camera/image and output-frame hypotheses carry matching poses and headers."""
    stamp_ns = 6_000_000_006
    tag_config = TagObjectConfig(
        tag_id=0,
        class_id='tagged_cube',
        object_name='red_cube',
        mesh_file_path='',
        confidence=0.99,
        dimensions=(0.05, 0.05, 0.05),
        tag_to_object_translation=(0.0, 0.0, -0.025),
        tag_to_object_rotation=(0.0, 0.0, 0.0, 1.0),
    )
    server = AprilTagObjectServer.__new__(AprilTagObjectServer)
    server._lock = RLock()
    server._frame_policy = resolve_pose_frame(
        'camera_optical_frame',
        'base_link',
    )
    server._tag_map = TagMapConfig(
        tag_family='tag36h11',
        tag_size=0.04,
        objects={0: tag_config},
    )
    server._ttl_ns = 1_000_000_000
    server._future_tolerance_ns = 0
    server._min_tag_edge_px = 1.0
    server._pose_filter = StablePoseFilter(
        min_stable_frames=1,
        max_translation_jump_m=0.02,
        max_rotation_jump_deg=5.0,
    )
    server._objects = {}
    server._metadata = {
        0: ObjectMetadata(name='red_cube', mesh_file_path=''),
    }
    server.get_clock = lambda: FakeClock(stamp_ns)
    server.get_logger = lambda: FakeLogger()
    message = _detection_array(stamp_ns)

    pose_transform = _transform(stamp_ns)
    pose_transform.transform.translation.x = 1.0
    pose_transform.transform.rotation.z = sqrt(0.5)
    pose_transform.transform.rotation.w = sqrt(0.5)
    server._process_detections(message, pose_transform)
    result = server._object_info(0, server._objects[0])

    camera_pose = result.detection_2d.results[0].pose.pose
    assert result.detection_2d.header.frame_id == 'camera_optical_frame'
    assert result.detection_2d.header.stamp == message.header.stamp
    assert (
        camera_pose.position.x,
        camera_pose.position.y,
        camera_pose.position.z,
    ) == pytest.approx((0.2, 0.0, 0.275))
    assert (
        camera_pose.orientation.x,
        camera_pose.orientation.y,
        camera_pose.orientation.z,
        camera_pose.orientation.w,
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0))

    output_pose = result.detection_3d.results[0].pose.pose
    assert result.detection_3d.header.frame_id == 'base_link'
    assert result.detection_3d.header.stamp == message.header.stamp
    assert (
        output_pose.position.x,
        output_pose.position.y,
        output_pose.position.z,
    ) == pytest.approx((1.0, 0.2, 0.275))
    assert (
        output_pose.orientation.x,
        output_pose.orientation.y,
        output_pose.orientation.z,
        output_pose.orientation.w,
    ) == pytest.approx((0.0, 0.0, sqrt(0.5), sqrt(0.5)))
    assert result.detection_3d.bbox.center == output_pose
