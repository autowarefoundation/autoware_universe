"""Coordinate transformation from ROS map (ENU) to re-centered tile-local.

The conversion pipeline mirrors ``splatsim.carla_integration.geo_transform``
but operates on **ENU** positions directly (no CARLA Y-flip needed because
the ROS map frame published by ``carla_ros.py`` is already ENU).

Pipeline (position)::

    ROS map  (x=East, y=North, z=Up)
      → MGRS absolute  (+ offset from proj_origin)
      → LLA             (lanelet2 MGRSProjector.reverse)
      → ECEF            (pyproj WGS84)
      → tile-local      (R_tile^T @ (ecef − t_tile))
      → re-centered     (− scene_origin)

Pipeline (rotation)::

    R_enu  →  R_enu_to_tile @ R_enu  →  R_tile
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2.core
import lanelet2.io
import numpy as np
from numpy.typing import NDArray
from pyproj import Transformer
import rclpy

_rlog = rclpy.logging.get_logger("splatsim_coord")


def _enu_to_ecef_rotation(lat_deg: float, lon_deg: float) -> NDArray[np.float64]:
    """Rotation matrix from ENU to ECEF at the given lat/lon (degrees)."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    slat, clat = np.sin(lat), np.cos(lat)
    slon, clon = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-slon, -slat * clon, clat * clon],
            [clon, -slat * slon, clat * slon],
            [0.0, clat, slat],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_quaternion_wxyz(
    R: NDArray[np.float64],
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to ``(w, x, y, z)`` quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


def _quaternion_xyzw_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> NDArray[np.float64]:
    """Convert ROS quaternion ``(x, y, z, w)`` to a 3x3 rotation matrix."""
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return R


def parse_tileset_transform(
    tileset_path: str,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read the root tile's ECEF rotation & translation from ``tileset.json``.

    Returns ``(ecef_rotation_3x3, ecef_translation_3)`` both as float64.
    """
    with open(tileset_path) as f:
        data = json.load(f)

    # Navigate to root tile transform — 3D Tiles stores a flat 16-element
    # column-major array.
    root = data.get("root", data)
    transform_flat = root["transform"]
    T = np.array(transform_flat, dtype=np.float64).reshape(4, 4).T  # col→row major
    return T[:3, :3].copy(), T[:3, 3].copy()


class CoordinateTransformer:
    """Transform poses from ROS map (ENU) to re-centered tile-local.

    Parameters
    ----------
    proj_origin:
        ``(latitude, longitude)`` of the xodr GeoReference origin in
        decimal degrees (parsed from ``+lat_0`` / ``+lon_0``).
    ecef_rotation:
        3x3 rotation from tile-local to ECEF (from ``tileset.json``).
    ecef_translation:
        3-vector ECEF translation (from ``tileset.json``).
    scene_origin:
        3-vector subtracted for re-centering (from
        ``InitializeResponse.scene_origin``).
    """

    def __init__(
        self,
        proj_origin: tuple[float, float],
        ecef_rotation: NDArray[np.float64],
        ecef_translation: NDArray[np.float64],
        scene_origin: NDArray[np.float64],
    ) -> None:
        lat_0, lon_0 = proj_origin

        # Lanelet2 MGRS projector
        self._projector = MGRSProjector(lanelet2.io.Origin(lat_0, lon_0))

        # MGRS offset at the geographic origin
        origin_gps = lanelet2.core.GPSPoint(lat_0, lon_0, 0.0)
        origin_local = self._projector.forward(origin_gps)
        self._mgrs_offset_x = origin_local.x
        self._mgrs_offset_y = origin_local.y

        # pyproj: LLA → ECEF
        self._lla_to_ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)

        # Tileset: tile_local → ECEF:  p_ecef = R @ p_tile + t
        self._tile_R_inv = ecef_rotation.T  # ECEF → tile-local
        self._tile_t = ecef_translation

        self._scene_origin = scene_origin

        # Precompute ENU → tile-local rotation
        R_enu_to_ecef = _enu_to_ecef_rotation(lat_0, lon_0)
        self._R_enu_to_tile = self._tile_R_inv @ R_enu_to_ecef

        self._debug_logged = False

        _rlog.warn(
            f"CoordinateTransformer: origin=({lat_0:.6f}, {lon_0:.6f}), "
            f"mgrs_offset=({self._mgrs_offset_x:.1f}, {self._mgrs_offset_y:.1f}), "
            f"scene_origin=({scene_origin[0]:.1f}, {scene_origin[1]:.1f}, {scene_origin[2]:.1f})"
        )

    def enu_position_to_tile_local(
        self,
        x: float,
        y: float,
        z: float,
    ) -> NDArray[np.float64]:
        """Convert an ENU (ROS map) position to re-centered tile-local."""
        # 1. ENU → MGRS absolute
        mgrs_x = x + self._mgrs_offset_x
        mgrs_y = y + self._mgrs_offset_y

        # 2. MGRS → LLA
        mgrs_pt = lanelet2.core.BasicPoint3d(mgrs_x, mgrs_y, z)
        gps = self._projector.reverse(mgrs_pt)

        # 3. LLA → ECEF
        ex, ey, ez = self._lla_to_ecef.transform(gps.lon, gps.lat, z)
        ecef = np.array([ex, ey, ez], dtype=np.float64)

        # 4. ECEF → tile-local
        tile_local = self._tile_R_inv @ (ecef - self._tile_t)

        # 5. Re-center
        result = tile_local - self._scene_origin

        if not self._debug_logged:
            self._debug_logged = True
            _rlog.warn(
                f"enu_position_to_tile_local:\n"
                f"  ENU input:  ({x:.4f}, {y:.4f}, {z:.4f})\n"
                f"  MGRS abs:   ({mgrs_x:.4f}, {mgrs_y:.4f})\n"
                f"  LLA:        (lat={gps.lat:.8f}, lon={gps.lon:.8f}, alt={z:.4f})\n"
                f"  ECEF:       ({ex:.4f}, {ey:.4f}, {ez:.4f})\n"
                f"  tile_local: ({tile_local[0]:.4f}, {tile_local[1]:.4f}, {tile_local[2]:.4f})\n"
                f"  scene_orig: ({self._scene_origin[0]:.4f}, {self._scene_origin[1]:.4f}, {self._scene_origin[2]:.4f})\n"
                f"  result:     ({result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f})"
            )

        return result

    def enu_rotation_to_tile_local(
        self,
        R_enu: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Convert an ENU rotation matrix to tile-local."""
        return self._R_enu_to_tile @ R_enu

    def transform_pose(
        self,
        position_enu: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Convert a full ROS pose to tile-local ``(position, quat_wxyz)``.

        Parameters
        ----------
        position_enu:
            ``(x, y, z)`` in ROS map / ENU frame.
        quaternion_xyzw:
            ``(x, y, z, w)`` quaternion (ROS convention).

        Returns
        -------
        tuple
            ``((x, y, z), (w, x, y, z))`` in re-centered tile-local.
        """
        tile_pos = self.enu_position_to_tile_local(*position_enu)

        R_enu = _quaternion_xyzw_to_rotation_matrix(*quaternion_xyzw)
        R_tile = self.enu_rotation_to_tile_local(R_enu)
        quat_wxyz = _rotation_matrix_to_quaternion_wxyz(R_tile)

        return (
            (float(tile_pos[0]), float(tile_pos[1]), float(tile_pos[2])),
            quat_wxyz,
        )
