"""SplatSimRGBCamera -- one Docker container + gRPC stream per RGB camera."""

from __future__ import annotations

import logging
import math
from pathlib import Path

import carla
import numpy as np

from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.coordinate_transformer import (
    CoordinateTransformer,
    _rotation_matrix_to_quaternion_wxyz,
)
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2
import rclpy

_rlog = rclpy.logging.get_logger("splatsim_camera")

# CARLA → ENU: flip y-axis (CARLA South → ENU North)
_S_CARLA_TO_ENU = np.diag([1.0, -1.0, 1.0])


def _fov_to_intrinsics(
    width: int, height: int, fov_deg: float,
) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to ``(fx, fy, cx, cy)``."""
    fov_rad = math.radians(fov_deg)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


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
        tileset_path: str,
        splatsim_image: str = "splatsim:latest",
        grpc_port: int = 50051,
        use_sh: bool = True,
        frame_rate: float = 20.0,
        image_topic: str = "/splatsim/image_raw",
        camera_info_topic: str = "/splatsim/camera_info",
        frame_id: str = "splatsim_camera",
        near_plane: float = 0.01,
        far_plane: float = 1000.0,
        device: str = "cuda:0",
        restart_container: bool = False,
        compress_format: str = "",
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        # Camera intrinsics from objects.json
        cam_w = sensor_spec["image_size_x"]
        cam_h = sensor_spec["image_size_y"]
        cam_fov = sensor_spec["fov"]

        # Camera extrinsic (actor -> camera) as CARLA Transform
        sp = sensor_spec["spawn_point"]
        self._sensor_transform = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp["roll"], pitch=sp["pitch"], yaw=sp["yaw"]),
        )

        # ── Docker container ──
        container_name = f"splatsim_{self._sensor_id}"
        self._docker = SplatSimDockerManager(
            image=splatsim_image, grpc_port=grpc_port,
            container_name=container_name,
            force_restart=restart_container,
        )
        self._docker.start(tileset_path)
        self._docker.wait_for_ready(timeout=120.0)

        # ── gRPC Initialize ──
        fx, fy, cx, cy = _fov_to_intrinsics(cam_w, cam_h, cam_fov)
        container_tileset = f"/data/{Path(tileset_path).name}"

        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)
        init_request = pb2.InitializeRequest(
            tileset_path=container_tileset,
            use_sh=use_sh,
            intrinsics=pb2.CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy, width=cam_w, height=cam_h,
            ),
            frame_rate=frame_rate,
            image_topic=image_topic,
            camera_info_topic=camera_info_topic,
            frame_id=frame_id,
            near_plane=near_plane,
            far_plane=far_plane,
            device=device,
            background_color=pb2.Vector3(x=0.0, y=0.0, z=0.0),
            compress_format=compress_format,
        )
        resp = self._grpc.initialize(init_request)
        if not resp.success:
            raise RuntimeError(f"splatsim Initialize failed: {resp.message}")

        # ── Coordinate transformer (values from gRPC server) ──
        scene_origin = np.array(
            [resp.scene_origin.x, resp.scene_origin.y, resp.scene_origin.z],
            dtype=np.float64,
        )
        if resp.ecef_rotation:
            ecef_rot = np.array(resp.ecef_rotation, dtype=np.float64).reshape(3, 3)
            ecef_trans = np.array(
                [resp.ecef_translation.x, resp.ecef_translation.y, resp.ecef_translation.z],
                dtype=np.float64,
            )
            _rlog.warn("Using ECEF transform from gRPC server")
        else:
            from autoware_carla_interface.splatsim.coordinate_transformer import (
                parse_tileset_transform,
            )
            ecef_rot, ecef_trans = parse_tileset_transform(tileset_path)
            _rlog.warn("gRPC server did not return ECEF transform; "
                       "falling back to parse_tileset_transform")
        _rlog.warn(
            f"ECEF rot:\n{ecef_rot}\n"
            f"ECEF trans: {ecef_trans}\n"
            f"scene_origin: {scene_origin}"
        )
        self._transformer = CoordinateTransformer(
            proj_origin=proj_origin,
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

        # ── Start streaming ──
        self._grpc.start_stream()
        self._update_count = 0
        _rlog.warn(f"SplatSimRGBCamera '{self._sensor_id}' initialized")

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
            self._sensor_transform.get_matrix(), dtype=np.float64,
        )
        T_carla_cam = T_world_actor @ T_actor_camera

        carla_pos = T_carla_cam[:3, 3]
        R_carla = T_carla_cam[:3, :3]

        # Position: CARLA → ENU (flip y) → tile-local
        enu_pos = _S_CARLA_TO_ENU @ carla_pos
        tile_pos = self._transformer.enu_position_to_tile_local(
            enu_pos[0], enu_pos[1], enu_pos[2],
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
