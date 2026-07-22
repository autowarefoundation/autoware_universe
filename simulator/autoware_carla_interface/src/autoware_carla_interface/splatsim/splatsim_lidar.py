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

from pathlib import Path

import carla
import numpy as np

from autoware_carla_interface.splatsim.coordinate_transformer import (
    CoordinateTransformer,
    _rotation_matrix_to_quaternion_wxyz,
)
from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2
import rclpy

_rlog = rclpy.logging.get_logger("splatsim_lidar")

# CARLA -> ENU: flip y-axis (CARLA South -> ENU North)
_S_CARLA_TO_ENU = np.diag([1.0, -1.0, 1.0])
# CARLA (Y-right) <-> ROS (Y-left) similarity for a 4x4 rigid transform.
_S4_CARLA_TO_ROS = np.diag([1.0, -1.0, 1.0, 1.0])


class SplatSimLidar:
    """Manages one splatsim Docker container for a single LiDAR sensor."""

    def __init__(
        self,
        sensor_spec: dict,
        *,
        proj_origin: tuple[float, float],
        tileset_path: str,
        splatsim_image: str = "splatsim:latest",
        grpc_port: int = 50061,
        use_sh: bool = True,
        sensor_type: str = "OT128",
        fps: float = 10.0,
        n_rows: int = 0,
        n_columns: int = 0,
        min_range_m: float = 0.0,
        max_range_m: float = 0.0,
        elevation_deg: tuple[float, ...] = (),
        drop_threshold: float = 0.0,
        alpha_threshold: float = 0.0,
        pointcloud_topic: str = "/sensing/lidar/top/pointcloud_before_sync",
        frame_id: str = "velodyne_top",
        device: str = "cuda:0",
        restart_container: bool = False,
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        # Sensor mount pose (actor-relative CARLA spawn) -> ROS base_link extrinsic
        sp = sensor_spec["spawn_point"]
        carla_tf = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp["roll"], pitch=sp["pitch"], yaw=sp["yaw"]),
        )
        t_carla = np.asarray(carla_tf.get_matrix(), dtype=np.float64)
        t_ros = _S4_CARLA_TO_ROS @ t_carla @ _S4_CARLA_TO_ROS
        ext_pos = t_ros[:3, 3]
        ext_quat_wxyz = _rotation_matrix_to_quaternion_wxyz(t_ros[:3, :3])

        # ── Docker container ──
        container_name = f"splatsim_{self._sensor_id}"
        self._docker = SplatSimDockerManager(
            image=splatsim_image, grpc_port=grpc_port,
            container_name=container_name,
            force_restart=restart_container,
        )
        self._docker.start(tileset_path)
        self._docker.wait_for_ready(timeout=120.0)

        # ── gRPC Initialize (scene) ──
        container_tileset = f"/data/{Path(tileset_path).name}"
        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)
        init_request = pb2.InitializeRequest(
            tileset_path=container_tileset,
            use_sh=use_sh,
            frame_rate=fps,
            device=device,
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
        self._transformer = CoordinateTransformer(
            proj_origin=proj_origin,
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

        # ── gRPC InitializeLidar (sensor) ──
        lidar_request = pb2.InitializeLidarRequest(
            sensor=pb2.LidarSensorConfig(
                name=str(self._sensor_id),
                sensor_type=sensor_type,
                n_rows=int(n_rows),
                n_columns=int(n_columns),
                fps=fps,
                min_range_m=min_range_m,
                max_range_m=max_range_m,
                extrinsic=pb2.Pose(
                    position=pb2.Vector3(
                        x=float(ext_pos[0]), y=float(ext_pos[1]), z=float(ext_pos[2]),
                    ),
                    rotation=pb2.Quaternion(
                        w=ext_quat_wxyz[0], x=ext_quat_wxyz[1],
                        y=ext_quat_wxyz[2], z=ext_quat_wxyz[3],
                    ),
                ),
                elevation_deg=list(elevation_deg),
                pointcloud_topic=pointcloud_topic,
                frame_id=frame_id,
                drop_threshold=drop_threshold,
                alpha_threshold=alpha_threshold,
            )
        )
        lidar_resp = self._grpc.initialize_lidar(lidar_request)
        if not lidar_resp.success:
            raise RuntimeError(f"splatsim InitializeLidar failed: {lidar_resp.message}")

        _rlog.warn(
            f"SplatSimLidar '{self._sensor_id}' initialized "
            f"(type={sensor_type}, topic={pointcloud_topic}, frame={frame_id})\n"
            f"  extrinsic pos=({ext_pos[0]:.4f}, {ext_pos[1]:.4f}, {ext_pos[2]:.4f})"
        )

        # ── Start streaming ──
        self._grpc.start_lidar_stream()
        self._update_count = 0

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
            enu_pos[0], enu_pos[1], enu_pos[2],
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
