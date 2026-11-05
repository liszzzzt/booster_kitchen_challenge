"""Minimal Booster Studio extension test for the torso-mounted E1R LiDAR."""
# copy to: 1rmk9-t2:/usr/local/booster_robot/booster_robocup_sim/extensions/e1r_3d_lidar_control.py

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
    """Synchronize a private MuJoCo model and publish a nine-ray test cloud."""

    def __init__(self, model: Any, data: Any, context: ExtensionContext):
        if model is None or data is None:
            raise RuntimeError("E1R extension requires a MuJoCo model")

        self._model = model
        self._data = data
        self._physics_client = context.get_service("physics_client")
        self._node = None
        self._publisher = None
        self._owns_rclpy = False
        self._last_scan_time = None
        self._first_publish = True

        self._site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "e1r_sensor",
        )
        self._trunk_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "trunk",
        )
        if self._site_id < 0:
            raise RuntimeError("MuJoCo site 'e1r_sensor' was not found")
        if self._trunk_id < 0:
            raise RuntimeError("MuJoCo body 'trunk' was not found")

        self._directions = self._make_test_directions()
        self._geom_groups = np.ones(6, dtype=np.uint8)

    @staticmethod
    def _make_test_directions() -> list[np.ndarray]:
        directions = []
        for pitch_degrees in (-15.0, 0.0, 15.0):
            pitch = math.radians(pitch_degrees)
            for yaw_degrees in (-30.0, 0.0, 30.0):
                yaw = math.radians(yaw_degrees)
                directions.append(
                    np.array(
                        [
                            math.cos(pitch) * math.cos(yaw),
                            math.cos(pitch) * math.sin(yaw),
                            math.sin(pitch),
                        ],
                        dtype=np.float64,
                    )
                )
        return directions

    def on_start(self) -> None:
        if self._physics_client is None:
            raise RuntimeError("physics_client is unavailable")

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self._node = rclpy.create_node("e1r_3d_lidar_test")
        self._publisher = self._node.create_publisher(
            PointCloud2,
            "/e1r/test_points",
            qos_profile_sensor_data,
        )
        self._physics_client.subscribe_physics_state(self._on_physics_state)
        print("[E1R3DLidarControl] Started", flush=True)

    def _on_physics_state(self, state) -> None:
        simulation_time = float(state.time)
        if self._last_scan_time is not None:
            if simulation_time < self._last_scan_time:
                self._last_scan_time = None
            elif simulation_time - self._last_scan_time < 1.0:
                return
        self._last_scan_time = simulation_time

        self._data.qpos[:] = state.qpos
        self._data.qvel[:] = state.qvel
        self._data.time = simulation_time
        mujoco.mj_forward(self._model, self._data)

        points = self._cast_test_rays()
        header = Header()
        header.stamp.sec = int(simulation_time)
        header.stamp.nanosec = int((simulation_time - int(simulation_time)) * 1e9)
        header.frame_id = "trunk"
        self._publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points)
        )

        if self._first_publish:
            self._first_publish = False
            print(
                f"[E1R3DLidarControl] Published {len(points)} test points",
                flush=True,
            )

    def _cast_test_rays(self) -> list[tuple[float, float, float]]:
        origin = self._data.site_xpos[self._site_id].copy()
        sensor_rotation = self._data.site_xmat[self._site_id].reshape(3, 3)
        trunk_position = self._data.xpos[self._trunk_id]
        trunk_rotation = self._data.xmat[self._trunk_id].reshape(3, 3)
        geom_id = np.array([-1], dtype=np.int32)
        points = []

        for local_direction in self._directions:
            world_direction = sensor_rotation @ local_direction
            geom_id[0] = -1
            distance = mujoco.mj_ray(
                self._model,
                self._data,
                origin,
                world_direction,
                self._geom_groups,
                1,
                self._trunk_id,
                geom_id,
            )
            if 0.05 <= distance <= 10.0:
                world_point = origin + distance * world_direction
                trunk_point = trunk_rotation.T @ (world_point - trunk_position)
                points.append(tuple(float(value) for value in trunk_point))

        return points

    def on_stop(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        print("[E1R3DLidarControl] Stopped", flush=True)
