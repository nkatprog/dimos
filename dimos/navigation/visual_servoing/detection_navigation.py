# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dimos_lcm.sensor_msgs import CameraInfo as DimosLcmCameraInfo
import numpy as np

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d import Detection3DPC
from dimos.protocol.tf import LCMTF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DetectionNavigation:
    _target_distance_3d: float = 1.5  # meters to maintain from person
    _min_distance_3d: float = 0.8  # meters before backing up
    _max_linear_speed_3d: float = 0.5  # m/s
    _max_angular_speed_3d: float = 0.8  # rad/s
    _linear_gain_3d: float = 0.8
    _angular_gain_3d: float = 1.5

    _tf: LCMTF
    _camera_info: CameraInfo

    def __init__(self, tf: LCMTF, camera_info: CameraInfo) -> None:
        self._tf = tf
        self._camera_info = camera_info

    def compute_twist_for_detection_3d(
        self, pointcloud: PointCloud2, detection: Detection2DBBox, image: Image
    ) -> Twist | None:
        """Project a 2D detection to 3D using pointcloud and compute navigation twist.

        Args:
            detection: 2D detection with bounding box
            image: Current image frame

        Returns:
            Twist command to navigate towards the detection's 3D position.
        """

        # Get transform from world frame to camera optical frame
        world_to_optical = self._tf.get(
            "camera_optical", pointcloud.frame_id, image.ts, time_tolerance=1.0
        )
        if world_to_optical is None:
            logger.warning("Could not get camera transform")
            return None

        lcm_camera_info = DimosLcmCameraInfo()
        lcm_camera_info.K = self._camera_info.K
        lcm_camera_info.width = self._camera_info.width
        lcm_camera_info.height = self._camera_info.height

        # Project to 3D using the pointcloud
        detection_3d = Detection3DPC.from_2d(
            det=detection,
            world_pointcloud=pointcloud,
            camera_info=lcm_camera_info,
            world_to_optical_transform=world_to_optical,
            filters=[],  # Skip filtering for faster processing in follow loop
        )

        if detection_3d is None:
            logger.warning("3D projection failed")
            return None

        # Navigate towards the 3D center of the detection
        target_position = detection_3d.center
        logger.info(
            f"3D target position: ({target_position.x:.2f}, {target_position.y:.2f}, {target_position.z:.2f})"
        )

        return self._compute_twist_from_3d(target_position)

    def _compute_twist_from_3d(self, target_position: Vector3) -> Twist:
        """Compute twist command to navigate towards a 3D target position.

        The target position is in world frame. We use the robot's base_link transform
        to determine relative position and compute appropriate velocities.

        Args:
            target_position: 3D position of the target in world frame.

        Returns:
            Twist command for the robot.
        """
        # Get robot's current position in world frame
        robot_transform = self._tf.get("world", "base_link", time_tolerance=1.0)
        if robot_transform is None:
            logger.warning("Could not get robot transform for 3D navigation")
            return Twist.zero()

        robot_pos = robot_transform.translation

        # Compute vector from robot to target in world frame
        dx = target_position.x - robot_pos.x
        dy = target_position.y - robot_pos.y
        distance = np.sqrt(dx * dx + dy * dy)

        # Compute angle to target in world frame
        angle_to_target = np.arctan2(dy, dx)

        # Get robot's current heading from transform
        robot_yaw = robot_transform.rotation.to_euler().z

        # Angle error (how much to turn)
        angle_error = angle_to_target - robot_yaw
        # Normalize to [-pi, pi]
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Compute angular velocity (turn towards target)
        angular_z = angle_error * self._angular_gain_3d
        angular_z = float(
            np.clip(angular_z, -self._max_angular_speed_3d, self._max_angular_speed_3d)
        )

        # Compute linear velocity based on distance
        distance_error = distance - self._target_distance_3d

        if distance < self._min_distance_3d:
            # Too close, back up
            linear_x = -self._max_linear_speed_3d * 0.6
        else:
            # Move forward based on distance error, reduce speed when turning
            turn_factor = 1.0 - min(abs(angle_error) / np.pi, 0.7)
            linear_x = distance_error * self._linear_gain_3d * turn_factor
            linear_x = float(
                np.clip(linear_x, -self._max_linear_speed_3d, self._max_linear_speed_3d)
            )

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_z),
        )
