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

"""Expose AprilTag observations through Isaac ROS Manipulation interfaces."""

from copy import deepcopy
from dataclasses import dataclass
from math import isfinite
import os
from threading import RLock
from typing import Dict, Tuple

from geometry_msgs.msg import Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_manipulation_interfaces.action import GetObjectPose, GetObjects
from isaac_ros_manipulation_interfaces.msg import ObjectInfo
from isaac_ros_manipulation_interfaces.srv import (
    AddMeshToObject,
    AssignNameToObject,
    ClearObjects,
)
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection3D, ObjectHypothesisWithPose

from .observation_filter import (
    axis_aligned_bbox,
    is_fresh,
    minimum_tag_edge_px,
    StablePoseFilter,
)
from .pose_math import compose_pose
from .tag_config import load_tag_map, TagMapConfig, TagObjectConfig


Point2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class CachedObject:
    """A stable object observation expressed in the configured camera frame."""

    header: Header
    source_stamp_ns: int
    position: Vector3
    orientation: Quaternion
    corners: Tuple[Point2, Point2, Point2, Point2]


@dataclass
class ObjectMetadata:
    """Mutable metadata assigned through the manipulation services."""

    name: str
    mesh_file_path: str


def _stamp_to_nanoseconds(header: Header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _pose(position: Vector3, orientation: Quaternion) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


class AprilTagObjectServer(Node):
    """Adapt stable AprilTag detections to the official manipulation API."""

    def __init__(self) -> None:
        super().__init__('apriltag_object_server')

        tag_config_file = str(
            self.declare_parameter('tag_config_file', '').value
        ).strip()
        tag_config_path = str(
            self.declare_parameter('tag_config_path', '').value
        ).strip()
        config_path = tag_config_file or tag_config_path
        self._expected_camera_frame = str(
            self.declare_parameter('expected_camera_frame', '').value
        ).strip()
        detections_topic = str(
            self.declare_parameter('detections_topic', '/tag_detections').value
        ).strip()
        detection_topic_alias = str(
            self.declare_parameter('detection_topic', '').value
        ).strip()
        detection_topic = detection_topic_alias or detections_topic
        pose_ttl_sec = float(self.declare_parameter('pose_ttl_sec', 0.5).value)
        min_stable_frames = int(self.declare_parameter('min_stable_frames', 5).value)
        min_tag_edge_px = float(self.declare_parameter('min_tag_edge_px', 40.0).value)
        max_translation_jump_m = float(
            self.declare_parameter('max_translation_jump_m', 0.01).value
        )
        max_rotation_jump_deg = float(
            self.declare_parameter('max_rotation_jump_deg', 5.0).value
        )

        if not config_path:
            raise ValueError('tag_config_file or tag_config_path must be configured')
        if not self._expected_camera_frame:
            raise ValueError('expected_camera_frame must be configured')
        if not detection_topic:
            raise ValueError('detection_topic must be non-empty')
        if not isfinite(pose_ttl_sec) or pose_ttl_sec <= 0.0:
            raise ValueError('pose_ttl_sec must be greater than zero')
        if min_stable_frames < 1:
            raise ValueError('min_stable_frames must be at least one')
        if not isfinite(min_tag_edge_px) or min_tag_edge_px <= 0.0:
            raise ValueError('min_tag_edge_px must be greater than zero')
        if not isfinite(max_translation_jump_m) or max_translation_jump_m <= 0.0:
            raise ValueError('max_translation_jump_m must be greater than zero')
        if not isfinite(max_rotation_jump_deg) or not 0.0 < max_rotation_jump_deg <= 180.0:
            raise ValueError('max_rotation_jump_deg must be in (0, 180]')

        self._tag_map: TagMapConfig = load_tag_map(config_path)
        self._ttl_ns = int(pose_ttl_sec * 1_000_000_000)
        self._min_tag_edge_px = min_tag_edge_px
        self._pose_filter = StablePoseFilter(
            min_stable_frames=min_stable_frames,
            max_translation_jump_m=max_translation_jump_m,
            max_rotation_jump_deg=max_rotation_jump_deg,
        )
        self._objects: Dict[int, CachedObject] = {}
        self._metadata = {
            tag_id: self._default_metadata(tag_config)
            for tag_id, tag_config in self._tag_map.objects.items()
        }
        self._lock = RLock()
        callback_group = ReentrantCallbackGroup()

        detection_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            detection_topic,
            self._on_detections,
            detection_qos,
            callback_group=callback_group,
        )
        self._get_objects_server = ActionServer(
            self,
            GetObjects,
            '/get_objects',
            execute_callback=self._execute_get_objects,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=callback_group,
        )
        self._get_object_pose_server = ActionServer(
            self,
            GetObjectPose,
            '/get_object_pose',
            execute_callback=self._execute_get_object_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=callback_group,
        )
        self._add_mesh_service = self.create_service(
            AddMeshToObject,
            'add_mesh_to_object',
            self._add_mesh_to_object,
            callback_group=callback_group,
        )
        self._assign_name_service = self.create_service(
            AssignNameToObject,
            'assign_name_to_object',
            self._assign_name_to_object,
            callback_group=callback_group,
        )
        self._clear_objects_service = self.create_service(
            ClearObjects,
            'clear_objects',
            self._clear_objects,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f'Listening for {self._tag_map.tag_family} detections on {detection_topic}; '
            f'expected camera frame is {self._expected_camera_frame}'
        )

    @staticmethod
    def _default_metadata(config: TagObjectConfig) -> ObjectMetadata:
        return ObjectMetadata(
            name=config.object_name,
            mesh_file_path=config.mesh_file_path,
        )

    @staticmethod
    def _accept_goal(_goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _invalidate_tag(self, tag_id: int) -> None:
        self._pose_filter.reset(tag_id)
        self._objects.pop(tag_id, None)

    def _on_detections(self, message: AprilTagDetectionArray) -> None:
        if message.header.frame_id != self._expected_camera_frame:
            self.get_logger().error(
                'Rejecting AprilTag array in frame '
                f'{message.header.frame_id!r}; expected {self._expected_camera_frame!r}'
            )
            return

        source_stamp_ns = _stamp_to_nanoseconds(message.header)
        now_ns = self.get_clock().now().nanoseconds
        if not is_fresh(source_stamp_ns, now_ns, self._ttl_ns):
            self.get_logger().warning(
                'Rejecting AprilTag array with an expired or future header timestamp'
            )
            return

        seen_tag_ids = set()
        with self._lock:
            for detection in message.detections:
                tag_id = int(detection.id)
                if tag_id in seen_tag_ids:
                    self.get_logger().warning(
                        f'Ignoring duplicate AprilTag ID {tag_id} in one detection array'
                    )
                    continue
                seen_tag_ids.add(tag_id)

                tag_config = self._tag_map.objects.get(tag_id)
                if tag_config is None or detection.family != self._tag_map.tag_family:
                    continue

                corners = tuple(
                    (float(corner.x), float(corner.y)) for corner in detection.corners
                )
                if minimum_tag_edge_px(corners) < self._min_tag_edge_px:
                    self._invalidate_tag(tag_id)
                    continue

                tag_pose = detection.pose.pose.pose
                try:
                    object_position, object_orientation = compose_pose(
                        (
                            tag_pose.position.x,
                            tag_pose.position.y,
                            tag_pose.position.z,
                        ),
                        (
                            tag_pose.orientation.x,
                            tag_pose.orientation.y,
                            tag_pose.orientation.z,
                            tag_pose.orientation.w,
                        ),
                        tag_config.tag_to_object_translation,
                        tag_config.tag_to_object_rotation,
                    )
                    update = self._pose_filter.update(
                        tag_id, object_position, object_orientation
                    )
                except (TypeError, ValueError) as error:
                    self.get_logger().warning(
                        f'Rejecting invalid pose for AprilTag {tag_id}: {error}'
                    )
                    self._invalidate_tag(tag_id)
                    continue

                if update.reset:
                    self._objects.pop(tag_id, None)
                if update.stable_pose is None:
                    continue

                self._objects[tag_id] = CachedObject(
                    header=deepcopy(message.header),
                    source_stamp_ns=source_stamp_ns,
                    position=update.stable_pose.position,
                    orientation=update.stable_pose.orientation,
                    corners=corners,
                )

    def _fresh_object(self, object_id: int, now_ns: int) -> CachedObject | None:
        cached = self._objects.get(object_id)
        if cached is None:
            return None
        if not is_fresh(cached.source_stamp_ns, now_ns, self._ttl_ns):
            self._invalidate_tag(object_id)
            return None
        return cached

    @staticmethod
    def _hypothesis(
        config: TagObjectConfig,
        cached: CachedObject,
    ) -> ObjectHypothesisWithPose:
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = config.class_id
        hypothesis.hypothesis.score = config.confidence
        hypothesis.pose.pose = _pose(cached.position, cached.orientation)
        return hypothesis

    def _object_info(self, object_id: int, cached: CachedObject) -> ObjectInfo:
        config = self._tag_map.objects[object_id]
        metadata = self._metadata[object_id]
        x_min, y_min, x_max, y_max = axis_aligned_bbox(cached.corners)

        detection_2d = Detection2D()
        detection_2d.header = deepcopy(cached.header)
        detection_2d.id = str(object_id)
        detection_2d.bbox.center.position.x = (x_min + x_max) / 2.0
        detection_2d.bbox.center.position.y = (y_min + y_max) / 2.0
        detection_2d.bbox.center.theta = 0.0
        detection_2d.bbox.size_x = x_max - x_min
        detection_2d.bbox.size_y = y_max - y_min
        detection_2d.results.append(self._hypothesis(config, cached))

        detection_3d = Detection3D()
        detection_3d.header = deepcopy(cached.header)
        detection_3d.id = str(object_id)
        detection_3d.bbox.center = _pose(cached.position, cached.orientation)
        detection_3d.bbox.size.x = config.dimensions[0]
        detection_3d.bbox.size.y = config.dimensions[1]
        detection_3d.bbox.size.z = config.dimensions[2]
        detection_3d.results.append(self._hypothesis(config, cached))

        object_info = ObjectInfo()
        object_info.object_id = object_id
        object_info.detection_2d = detection_2d
        object_info.detection_3d = detection_3d
        object_info.has_segmentation_mask = False
        object_info.mesh_file_path = metadata.mesh_file_path
        object_info.name = metadata.name
        return object_info

    def _execute_get_objects(self, goal_handle) -> GetObjects.Result:
        result = GetObjects.Result()
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            for object_id in sorted(self._objects):
                cached = self._fresh_object(object_id, now_ns)
                if cached is not None:
                    result.objects.append(self._object_info(object_id, cached))
        goal_handle.succeed()
        return result

    def _execute_get_object_pose(self, goal_handle) -> GetObjectPose.Result:
        result = GetObjectPose.Result()
        object_id = int(goal_handle.request.object_id)
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            cached = self._fresh_object(object_id, now_ns)
            if cached is None:
                self.get_logger().warning(
                    f'Object {object_id} has no stable, fresh AprilTag pose'
                )
                goal_handle.abort()
                return result
            result.object_pose = _pose(cached.position, cached.orientation)
        goal_handle.succeed()
        return result

    def _add_mesh_to_object(self, request, response):
        object_ids = list(request.object_ids)
        mesh_paths = list(request.mesh_file_paths)
        if len(object_ids) != len(mesh_paths):
            response.success = False
            response.failed_ids = object_ids
            response.message = 'object_ids and mesh_file_paths must have equal length'
            return response

        failed_ids = []
        with self._lock:
            for object_id, mesh_path in zip(object_ids, mesh_paths):
                normalized_path = str(mesh_path).strip()
                if (
                    object_id not in self._metadata or
                    not os.path.isfile(normalized_path) or
                    not os.access(normalized_path, os.R_OK)
                ):
                    failed_ids.append(object_id)
                    continue
                self._metadata[object_id].mesh_file_path = normalized_path

        response.failed_ids = failed_ids
        response.success = not failed_ids
        response.message = (
            'Mesh paths assigned successfully'
            if response.success else 'One or more object IDs or mesh files were invalid'
        )
        return response

    def _assign_name_to_object(self, request, response):
        object_name = str(request.name).strip()
        with self._lock:
            if request.object_id not in self._metadata or not object_name:
                response.result = False
                return response
            self._metadata[request.object_id].name = object_name
        response.result = True
        return response

    def _clear_objects(self, request, response):
        with self._lock:
            if request.object_ids:
                object_ids = {int(object_id) for object_id in request.object_ids}
                count = sum(object_id in self._objects for object_id in object_ids)
                for object_id in object_ids:
                    self._invalidate_tag(object_id)
                    config = self._tag_map.objects.get(object_id)
                    if config is not None:
                        self._metadata[object_id] = self._default_metadata(config)
            else:
                count = len(self._objects)
                self._objects.clear()
                self._pose_filter.clear()
                self._metadata = {
                    tag_id: self._default_metadata(config)
                    for tag_id, config in self._tag_map.objects.items()
                }
        response.count = count
        return response


def main(args=None) -> None:
    """Run the AprilTag object server with a multithreaded executor."""
    rclpy.init(args=args)
    node = AprilTagObjectServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
