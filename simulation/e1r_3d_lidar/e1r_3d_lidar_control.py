"""RoboSense E1R 3D LiDAR simulation for Booster Studio."""

import math
from typing import Any

import mujoco
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from core.extension_context import ExtensionContext


class E1R3DLidarControl:
    """Publish raw MuJoCo ray returns as a ROS 2 point cloud."""

    FREQUENCY_HZ = 10.0
    HORIZONTAL_FOV_DEGREES = 120.0
    VERTICAL_FOV_DEGREES = 90.0
    HORIZONTAL_RAYS = 181
    VERTICAL_RAYS = 144
    RANGE_MIN = 0.1
    RANGE_MAX = 75.0

    def __init__(self, model: Any, data: Any, context: ExtensionContext):
        self._model = model
        self._data = data
        self._physics_client = context.get_service("physics_client")
        self._node = None
        self._publisher = None
        self._last_scan_time = None

        self._site_id = self._find_scoped_id(
            mujoco.mjtObj.mjOBJ_SITE,
            model.nsite,
            "e1r_sensor",
        )
        self._trunk_id = self._find_scoped_id(
            mujoco.mjtObj.mjOBJ_BODY,
            model.nbody,
            "trunk",
        )
        self._local_directions = self._make_directions()
        self._ray_count = len(self._local_directions)
        self._geom_groups = np.ones(6, dtype=np.uint8)
        self._geom_ids = np.empty(self._ray_count, dtype=np.int32)
        self._distances = np.empty(self._ray_count, dtype=np.float64)

    def _find_scoped_id(self, object_type, object_count: int, name: str) -> int:
        object_id = mujoco.mj_name2id(self._model, object_type, name)
        if object_id >= 0:
            return object_id

        suffix = f"_{name}"
        for candidate_id in range(object_count):
            candidate_name = mujoco.mj_id2name(
                self._model,
                object_type,
                candidate_id,
            )
            if candidate_name and candidate_name.endswith(suffix):
                return candidate_id
        return -1

    def _make_directions(self) -> np.ndarray:
        yaw = np.linspace(
            -math.radians(self.HORIZONTAL_FOV_DEGREES) / 2.0,
            math.radians(self.HORIZONTAL_FOV_DEGREES) / 2.0,
            self.HORIZONTAL_RAYS,
        )
        pitch = np.linspace(
            -math.radians(self.VERTICAL_FOV_DEGREES) / 2.0,
            math.radians(self.VERTICAL_FOV_DEGREES) / 2.0,
            self.VERTICAL_RAYS,
        )
        yaw_grid, pitch_grid = np.meshgrid(yaw, pitch)
        cos_pitch = np.cos(pitch_grid)
        return np.column_stack(
            (
                (cos_pitch * np.cos(yaw_grid)).ravel(),
                (cos_pitch * np.sin(yaw_grid)).ravel(),
                np.sin(pitch_grid).ravel(),
            )
        ).astype(np.float64)

    def on_start(self) -> None:
        rclpy.init()
        self._node = rclpy.create_node("e1r_3d_lidar")
        self._publisher = self._node.create_publisher(
            PointCloud2,
            "/e1r/points",
            qos_profile_sensor_data,
        )
        self._physics_client.subscribe_physics_state(self._on_physics_state)

    def _on_physics_state(self, state) -> None:
        simulation_time = float(state.time)
        if self._last_scan_time is not None:
            if simulation_time < self._last_scan_time:
                self._last_scan_time = None
            elif simulation_time - self._last_scan_time < 1.0 / self.FREQUENCY_HZ:
                return
        self._last_scan_time = simulation_time

        self._data.qpos[:] = state.qpos
        self._data.qvel[:] = state.qvel
        self._data.time = simulation_time
        mujoco.mj_forward(self._model, self._data)

        points = self._cast_rays()
        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        header.frame_id = "e1r_link"
        self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _cast_rays(self) -> np.ndarray:
        origin = self._data.site_xpos[self._site_id].copy()
        sensor_rotation = self._data.site_xmat[self._site_id].reshape(3, 3)
        world_directions = self._local_directions @ sensor_rotation.T

        mujoco.mj_multiRay(
            self._model,
            self._data,
            origin,
            world_directions.ravel(),
            self._geom_groups,
            1,
            self._trunk_id,
            self._geom_ids,
            self._distances,
            None,
            self._ray_count,
            self.RANGE_MAX,
        )

        valid = (self._distances >= self.RANGE_MIN) & (
            self._distances <= self.RANGE_MAX
        )
        return (
            self._distances[valid, None] * self._local_directions[valid]
        ).astype(np.float32)

    def on_stop(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()
