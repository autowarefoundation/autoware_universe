# Copyright 2024 Tier IV, Inc.
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

"""SplatSimLidar -- one Docker container + gRPC stream per LiDAR sensor.

Replaces a CARLA ``sensor.lidar.ray_cast`` sensor with Gaussian Splatting
rendering.  The Docker container publishes ``sensor_msgs/PointCloud2`` messages
directly via CycloneDDS on the sensor's sensing topic.

Coordinate contract (splatsim v2 / v25 ENU usdz, verified against the server):

* World frame is Z-up ENU, recentred to the scene centroid.  The client sends
  the **base_link -> world** pose in ROS convention (X-forward, Y-left, Z-up),
  tile-local.  Unlike the camera path there is **no RDF remap**.
* The **sensor -> base_link** extrinsic is sent once at ``InitializeLidar``.
  The server composes ``sensor_to_world = base_to_world @ extrinsic`` internally,
  so the stream only carries the moving base_link pose.

To reproduce the exact viewpoint of the real CARLA LiDAR the extrinsic is taken
from the same actor-relative spawn pose CARLA used (``cfg.transform``), converted
from CARLA (Y-right) to ROS base_link (Y-left).  Because this integration
publishes the CARLA actor pose *as* base_link (see ``_publish_localization``),
the streamed pose is the actor pose and the extrinsic stays actor-relative --
identical placement to the CARLA sensor.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from autoware_carla_interface.splatsim.coordinate_transformer import (
    _rotation_matrix_to_quaternion_wxyz,
)
from autoware_carla_interface.splatsim.coordinate_transformer import CoordinateTransformer
from autoware_carla_interface.splatsim.coordinate_transformer import resolve_ecef_transform
from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2
import carla
import numpy as np
import rclpy

_rlog = rclpy.logging.get_logger("splatsim_lidar")

# CARLA -> ENU: flip y-axis (CARLA South -> ENU North)
_S_CARLA_TO_ENU = np.diag([1.0, -1.0, 1.0])
# CARLA (Y-right) <-> ROS (Y-left) similarity for a 4x4 rigid transform.
_S4_CARLA_TO_ROS = np.diag([1.0, -1.0, 1.0, 1.0])


@dataclass(frozen=True)
class SplatSimLidarConfig:
    """Tunable parameters for a :class:`SplatSimLidar`."""

    tileset_path: str
    splatsim_image: str = "splatsim:latest"
    grpc_port: int = 50061
    use_sh: bool = True
    # Defaults mirror the real Velodyne HDL-64E sensor spec (CARLA's default
    # LiDAR: min range 0.9 m, max range 120 m, 64 channels, 0.1728 deg
    # horizontal resolution at 10 Hz). The built-in "HDL64E" table supplies the
    # actual per-channel vertical angles, so elevation_deg is left empty.
    sensor_type: str = "HDL64E"
    fps: float = 10.0
    n_rows: int = 64
    n_columns: int = 2083
    min_range_m: float = 0.9
    max_range_m: float = 120.0
    elevation_deg: tuple[float, ...] = ()
    drop_threshold: float = 0.5
    alpha_threshold: float = 0.1
    pointcloud_topic: str = "/sensing/lidar/top/pointcloud_before_sync"
    frame_id: str = "velodyne_top"
    device: str = "cuda:0"
    restart_container: bool = False


class SplatSimLidar:
    """Manages one splatsim Docker container for a single LiDAR sensor."""

    def __init__(
        self,
        sensor_spec: dict,
        *,
        proj_origin: tuple[float, float],
        config: SplatSimLidarConfig,
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        ext_pos, ext_quat_wxyz = self._sensor_extrinsic(sensor_spec["spawn_point"])
        self._docker = self._start_container(config)
        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)
        resp = self._initialize_scene(config)
        self._transformer = self._build_transformer(resp, config, proj_origin)
        self._initialize_lidar(config, ext_pos, ext_quat_wxyz)

        # ── Start streaming ──
        self._grpc.start_lidar_stream()
        self._update_count = 0

    @staticmethod
    def _sensor_extrinsic(sp: dict):
        """Convert an actor-relative CARLA spawn pose to a ROS base_link extrinsic."""
        carla_tf = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp["roll"], pitch=sp["pitch"], yaw=sp["yaw"]),
        )
        t_carla = np.asarray(carla_tf.get_matrix(), dtype=np.float64)
        t_ros = _S4_CARLA_TO_ROS @ t_carla @ _S4_CARLA_TO_ROS
        return t_ros[:3, 3], _rotation_matrix_to_quaternion_wxyz(t_ros[:3, :3])

    def _start_container(self, config: SplatSimLidarConfig) -> SplatSimDockerManager:
        docker = SplatSimDockerManager(
            image=config.splatsim_image,
            grpc_port=config.grpc_port,
            container_name=f"splatsim_{self._sensor_id}",
            force_restart=config.restart_container,
        )
        docker.start(config.tileset_path)
        docker.wait_for_ready(timeout=120.0)
        return docker

    def _initialize_scene(self, config: SplatSimLidarConfig):
        init_request = pb2.InitializeRequest(
            scene_path=f"/data/{Path(config.tileset_path).name}",
            use_sh=config.use_sh,
            frame_rate=config.fps,
            device=config.device,
        )
        resp = self._grpc.initialize(init_request)
        if not resp.success:
            raise RuntimeError(f"splatsim Initialize failed: {resp.message}")
        return resp

    def _build_transformer(
        self, resp, config: SplatSimLidarConfig, proj_origin: tuple[float, float]
    ) -> CoordinateTransformer:
        scene_origin = np.array(
            [resp.scene_origin.x, resp.scene_origin.y, resp.scene_origin.z],
            dtype=np.float64,
        )
        ecef_rot, ecef_trans = resolve_ecef_transform(resp, config.tileset_path)
        return CoordinateTransformer(
            proj_origin=proj_origin,
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

    def _initialize_lidar(self, config: SplatSimLidarConfig, ext_pos, ext_quat_wxyz) -> None:
        lidar_request = pb2.InitializeLidarRequest(
            sensor=pb2.LidarSensorConfig(
                name=str(self._sensor_id),
                sensor_type=config.sensor_type,
                n_rows=int(config.n_rows),
                n_columns=int(config.n_columns),
                fps=config.fps,
                min_range_m=config.min_range_m,
                max_range_m=config.max_range_m,
                extrinsic=pb2.Pose(
                    position=pb2.Vector3(
                        x=float(ext_pos[0]),
                        y=float(ext_pos[1]),
                        z=float(ext_pos[2]),
                    ),
                    rotation=pb2.Quaternion(
                        w=ext_quat_wxyz[0],
                        x=ext_quat_wxyz[1],
                        y=ext_quat_wxyz[2],
                        z=ext_quat_wxyz[3],
                    ),
                ),
                elevation_deg=list(config.elevation_deg),
                pointcloud_topic=config.pointcloud_topic,
                frame_id=config.frame_id,
                drop_threshold=config.drop_threshold,
                alpha_threshold=config.alpha_threshold,
            )
        )
        lidar_resp = self._grpc.initialize_lidar(lidar_request)
        if not lidar_resp.success:
            raise RuntimeError(f"splatsim InitializeLidar failed: {lidar_resp.message}")

        _rlog.warn(
            f"SplatSimLidar '{self._sensor_id}' initialized "
            f"(type={config.sensor_type}, topic={config.pointcloud_topic}, "
            f"frame={config.frame_id})\n"
            f"  extrinsic pos=({ext_pos[0]:.4f}, {ext_pos[1]:.4f}, {ext_pos[2]:.4f})"
        )

    def update(
        self,
        actor_matrix_4x4: list[list[float]],
        stamp_sec: int,
        stamp_nanosec: int,
    ) -> None:
        """Send the base_link (ego actor) pose in tile-local ROS coords to splatsim.

        Parameters
        ----------
        actor_matrix_4x4 : list[list[float]]
            Raw 4x4 world-to-actor matrix from ``carla.Transform.get_matrix()``.
            The ego actor pose is published as base_link by this integration.
        """
        t_world_actor = np.array(actor_matrix_4x4, dtype=np.float64)
        carla_pos = t_world_actor[:3, 3]
        r_carla = t_world_actor[:3, :3]

        # Position: CARLA -> ENU (flip y) -> tile-local
        enu_pos = _S_CARLA_TO_ENU @ carla_pos
        tile_pos = self._transformer.enu_position_to_tile_local(
            enu_pos[0],
            enu_pos[1],
            enu_pos[2],
        )

        # Rotation: CARLA -> ENU -> tile-local -> ROS base_link (X-fwd, Y-left, Z-up).
        # R_tile columns are the physical [fwd, right, up] axes in the world frame
        # (det=-1 from the Y-flip).  Negating the "right" column yields "left" and
        # restores det=+1, giving a proper base_link->world rotation (no RDF remap).
        r_enu = _S_CARLA_TO_ENU @ r_carla
        r_tile = self._transformer.enu_rotation_to_tile_local(r_enu)
        r_b2w = np.column_stack([r_tile[:, 0], -r_tile[:, 1], r_tile[:, 2]])
        quat_wxyz = _rotation_matrix_to_quaternion_wxyz(r_b2w)

        self._update_count += 1
        if self._update_count <= 10 or self._update_count % 50 == 0:
            _rlog.warn(
                f"LiDAR pose update #{self._update_count}:\n"
                f"  CARLA pos: ({carla_pos[0]:.4f}, {carla_pos[1]:.4f}, {carla_pos[2]:.4f})\n"
                f"  tile_pos:  ({tile_pos[0]:.4f}, {tile_pos[1]:.4f}, {tile_pos[2]:.4f})"
            )

        self._grpc.send_lidar_data(
            sec=stamp_sec,
            nanosec=stamp_nanosec,
            position=(float(tile_pos[0]), float(tile_pos[1]), float(tile_pos[2])),
            rotation_wxyz=quat_wxyz,
        )

    def shutdown(self) -> None:
        """Close gRPC stream and stop Docker container."""
        if self._grpc is None:
            return
        _rlog.warn(f"Shutting down SplatSimLidar '{self._sensor_id}'")
        self._grpc.close_lidar_stream()
        self._grpc.close()
        self._grpc = None
        self._docker.stop()
        self._docker = None

    def __del__(self) -> None:
        self.shutdown()
