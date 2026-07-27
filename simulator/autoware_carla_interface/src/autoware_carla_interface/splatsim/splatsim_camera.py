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

"""SplatSimRGBCamera -- one Docker container + gRPC stream per RGB camera."""

from __future__ import annotations

from dataclasses import dataclass
import math
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

_rlog = rclpy.logging.get_logger("splatsim_camera")

# CARLA → ENU: flip y-axis (CARLA South → ENU North)
_S_CARLA_TO_ENU = np.diag([1.0, -1.0, 1.0])


def _fov_to_intrinsics(
    width: int,
    height: int,
    fov_deg: float,
) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to ``(fx, fy, cx, cy)``."""
    fov_rad = math.radians(fov_deg)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


@dataclass(frozen=True)
class SplatSimCameraConfig:
    """Tunable parameters for a :class:`SplatSimRGBCamera`."""

    tileset_path: str
    splatsim_image: str = "splatsim:latest"
    grpc_port: int = 50051
    use_sh: bool = True
    frame_rate: float = 20.0
    image_topic: str = "/splatsim/image_raw"
    camera_info_topic: str = "/splatsim/camera_info"
    frame_id: str = "splatsim_camera"
    near_plane: float = 0.01
    far_plane: float = 1000.0
    device: str = "cuda:0"
    restart_container: bool = False
    compress_format: str = ""


class SplatSimRGBCamera:
    """Manages one splatsim Docker container for a single RGB camera.

    Replaces a CARLA ``sensor.camera.rgb`` sensor with Gaussian Splatting
    rendering.  The Docker container publishes ``Image`` and ``CameraInfo``
    messages directly via CycloneDDS.
    """

    def __init__(
        self,
        sensor_spec: dict,
        *,
        proj_origin: tuple[float, float],
        config: SplatSimCameraConfig,
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        # Camera extrinsic (actor -> camera) as CARLA Transform
        sp = sensor_spec["spawn_point"]
        self._sensor_transform = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp["roll"], pitch=sp["pitch"], yaw=sp["yaw"]),
        )

        self._docker = self._start_container(config)
        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)
        resp = self._initialize_render(sensor_spec, config)
        self._transformer = self._build_transformer(resp, config, proj_origin)

        # ── Start streaming ──
        self._grpc.start_stream()
        self._update_count = 0
        _rlog.warn(f"SplatSimRGBCamera '{self._sensor_id}' initialized")

    def _start_container(self, config: SplatSimCameraConfig) -> SplatSimDockerManager:
        docker = SplatSimDockerManager(
            image=config.splatsim_image,
            grpc_port=config.grpc_port,
            container_name=f"splatsim_{self._sensor_id}",
            force_restart=config.restart_container,
        )
        docker.start(config.tileset_path)
        docker.wait_for_ready(timeout=120.0)
        return docker

    def _initialize_render(self, sensor_spec: dict, config: SplatSimCameraConfig):
        fx, fy, cx, cy = _fov_to_intrinsics(
            sensor_spec["image_size_x"], sensor_spec["image_size_y"], sensor_spec["fov"]
        )
        init_request = pb2.InitializeRequest(
            scene_path=f"/data/{Path(config.tileset_path).name}",
            use_sh=config.use_sh,
            intrinsics=pb2.CameraIntrinsics(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                width=sensor_spec["image_size_x"],
                height=sensor_spec["image_size_y"],
            ),
            frame_rate=config.frame_rate,
            image_topic=config.image_topic,
            camera_info_topic=config.camera_info_topic,
            frame_id=config.frame_id,
            near_plane=config.near_plane,
            far_plane=config.far_plane,
            device=config.device,
            background_color=pb2.Vector3(x=0.0, y=0.0, z=0.0),
            compress_format=config.compress_format,
        )
        resp = self._grpc.initialize(init_request)
        if not resp.success:
            raise RuntimeError(f"splatsim Initialize failed: {resp.message}")
        return resp

    def _build_transformer(
        self, resp, config: SplatSimCameraConfig, proj_origin: tuple[float, float]
    ) -> CoordinateTransformer:
        scene_origin = np.array(
            [resp.scene_origin.x, resp.scene_origin.y, resp.scene_origin.z],
            dtype=np.float64,
        )
        ecef_rot, ecef_trans = resolve_ecef_transform(resp, config.tileset_path)
        _rlog.warn(
            f"ECEF rot:\n{ecef_rot}\n" f"ECEF trans: {ecef_trans}\n" f"scene_origin: {scene_origin}"
        )
        return CoordinateTransformer(
            proj_origin=proj_origin,
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

    def update(
        self,
        actor_matrix_4x4: list[list[float]],
        stamp_sec: int,
        stamp_nanosec: int,
    ) -> None:
        """Compute camera pose in tile-local coordinates and send to splatsim.

        Parameters
        ----------
        actor_matrix_4x4 : list[list[float]]
            Raw 4x4 world-to-actor matrix from ``carla.Transform.get_matrix()``.
        """
        T_world_actor = np.array(actor_matrix_4x4, dtype=np.float64)
        T_actor_camera = np.asarray(
            self._sensor_transform.get_matrix(),
            dtype=np.float64,
        )
        T_carla_cam = T_world_actor @ T_actor_camera

        carla_pos = T_carla_cam[:3, 3]
        R_carla = T_carla_cam[:3, :3]

        # Position: CARLA → ENU (flip y) → tile-local
        enu_pos = _S_CARLA_TO_ENU @ carla_pos
        tile_pos = self._transformer.enu_position_to_tile_local(
            enu_pos[0],
            enu_pos[1],
            enu_pos[2],
        )

        self._update_count += 1
        if self._update_count <= 10 or self._update_count % 50 == 0:
            _rlog.warn(
                f"Pose update #{self._update_count}:\n"
                f"  CARLA pos: ({carla_pos[0]:.4f}, {carla_pos[1]:.4f}, {carla_pos[2]:.4f})\n"
                f"  ENU pos:   ({enu_pos[0]:.4f}, {enu_pos[1]:.4f}, {enu_pos[2]:.4f})\n"
                f"  tile_pos:  ({tile_pos[0]:.4f}, {tile_pos[1]:.4f}, {tile_pos[2]:.4f})"
            )

        # Rotation: CARLA → ENU (flip y) → tile-local → RDF
        R_enu = _S_CARLA_TO_ENU @ R_carla
        R_tile = self._transformer.enu_rotation_to_tile_local(R_enu)
        # R_tile has det=-1 (from the Y-flip).  Apply RDF remapping here
        # (same as standalone _compute_viewmat) so the result is a proper
        # rotation (det=+1) that survives the quaternion roundtrip.
        #   gsplat X=right  ← camera Y = R_tile[:, 1]
        #   gsplat Y=down   ← camera -Z = -R_tile[:, 2]
        #   gsplat Z=forward ← camera X = R_tile[:, 0]
        R_rdf = np.column_stack([R_tile[:, 1], -R_tile[:, 2], R_tile[:, 0]])
        quat_wxyz = _rotation_matrix_to_quaternion_wxyz(R_rdf)

        self._grpc.send_camera_data(
            sec=stamp_sec,
            nanosec=stamp_nanosec,
            position=(float(tile_pos[0]), float(tile_pos[1]), float(tile_pos[2])),
            rotation_wxyz=quat_wxyz,
        )

    def shutdown(self) -> None:
        """Close gRPC stream and stop Docker container."""
        if self._grpc is None:
            return
        _rlog.warn(f"Shutting down SplatSimRGBCamera '{self._sensor_id}'")
        self._grpc.close_stream()
        self._grpc.close()
        self._grpc = None
        self._docker.stop()
        self._docker = None

    def __del__(self) -> None:
        self.shutdown()
