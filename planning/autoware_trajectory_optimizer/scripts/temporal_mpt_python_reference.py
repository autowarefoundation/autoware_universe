# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import time
from typing import Optional

from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32


def _quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _find_acados_mpt_root() -> Path:
    env = os.environ.get("AUTOWARE_ACADOS_MPT_ROOT")
    if env:
        p = Path(env)
        if (p / "generators" / "utils.py").is_file():
            return p.resolve()
    # Development: .../autoware_trajectory_optimizer/scripts/<this>.py -> .../src/acados_mpt
    here = Path(__file__).resolve().parent
    pkg_root = here.parent
    cand = pkg_root / "src" / "acados_mpt"
    if (cand / "generators" / "utils.py").is_file():
        return cand.resolve()
    # Install layout or nested workspace: walk up until we find src/acados_mpt
    walk = here
    for _ in range(12):
        cand = walk / "src" / "acados_mpt"
        if (cand / "generators" / "utils.py").is_file():
            return cand.resolve()
        if walk.parent == walk:
            break
        walk = walk.parent
    raise RuntimeError(
        "Set AUTOWARE_ACADOS_MPT_ROOT to the acados_mpt directory (contains generators/). "
        "Required when the package is used from install space without a nearby source tree."
    )


def _setup_pythonpath(acados_mpt_root: Path) -> None:
    acados = os.environ.get("ACADOS_SOURCE_DIR", "/opt/acados")
    tpl = Path(acados) / "interfaces" / "acados_template"
    if not tpl.is_dir():
        raise RuntimeError(f"ACADOS_SOURCE_DIR invalid: missing {tpl}")
    root_s = str(acados_mpt_root.resolve())
    if root_s not in sys.path:
        sys.path.insert(0, root_s)
    tpl_s = str(tpl)
    if tpl_s not in sys.path:
        sys.path.insert(0, tpl_s)


def _default_solver_build_dir(acados_mpt_root: Path) -> Path:
    env = os.environ.get("ACADOS_MPT_PYTHON_SOLVER_DIR")
    if env:
        return Path(env).resolve()
    return (acados_mpt_root / "generators" / "_example_temporal_xyv_build").resolve()


def _load_solver(build_dir: Path, N: int, Tf: float):
    """Load ``AcadosOcpSolver`` from ``build_dir`` (must contain ``acados_ocp.json``).

    ``PathTrackingMPCTemporal(..., build=False)`` is **codegen-only** and always returns
    ``acados_solver is None`` (see ``path_tracking_mpc_temporal.py`` — that path is for CMake).
    We need ``build=True`` to construct the solver; ``generate=False`` reuses existing json/code.
    """
    from generators.path_tracking_mpc_temporal import PathTrackingMPCTemporal

    old = os.getcwd()
    try:
        os.chdir(build_dir)
        # build=True: actually create AcadosOcpSolver; generate=False: do not re-export ocp from Python
        mpc = PathTrackingMPCTemporal(Tf, N, build=True, generate=False)
    finally:
        os.chdir(old)
    solver = mpc.acados_solver
    if solver is None:
        raise RuntimeError(
            "Python acados solver is None after PathTrackingMPCTemporal(build=True). "
            "Ensure acados_ocp.json in the solver dir is valid; try once: "
            "cd $AUTOWARE_ACADOS_MPT_ROOT/generators && "
            "python3 example_track_xyv.py --open-loop --no-plot --N 50 --dt 0.1"
        )
    return solver


class TemporalMptPythonReference(Node):
    def __init__(self) -> None:
        super().__init__("temporal_mpt_python_reference")

        self.declare_parameter(
            "topic_prefix",
            "/planning/trajectory_generator/neural_network_based_planner/"
            "trajectory_optimizer_node/debug/temporal_mpt",
        )
        self.declare_parameter("N", 50)
        self.declare_parameter("Tf", 5.0)
        self.declare_parameter("lf", 1.0)
        self.declare_parameter("lr", 1.0)

        prefix = self.get_parameter("topic_prefix").get_parameter_value().string_value.rstrip("/")
        self._N = int(self.get_parameter("N").value)
        self._Tf = float(self.get_parameter("Tf").value)
        self._lf = float(self.get_parameter("lf").value)
        self._lr = float(self.get_parameter("lr").value)

        acados_mpt_root = _find_acados_mpt_root()
        _setup_pythonpath(acados_mpt_root)
        build_dir = _default_solver_build_dir(acados_mpt_root)
        json_path = build_dir / "acados_ocp.json"
        if not json_path.is_file():
            self.get_logger().fatal(
                f"Missing {json_path}. Build the Python solver (see script docstring)."
            )
            raise RuntimeError(f"missing {json_path}")

        self.get_logger().info(f"AUTOWARE_ACADOS_MPT_ROOT -> {acados_mpt_root}")
        self.get_logger().info(f"Python acados solver dir -> {build_dir}")

        from generators.utils import solve_autoware_temporal_mpc

        self._solve_autoware = solve_autoware_temporal_mpc
        self._solver = _load_solver(build_dir, self._N, self._Tf)

        self._last_odom: Optional[Odometry] = None

        TEMPORAL_MPT_DEBUG_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        out_base = f"{prefix}/python"
        self._pub_traj = self.create_publisher(
            Trajectory, f"{out_base}/output/trajectory", TEMPORAL_MPT_DEBUG_QOS
        )
        self._pub_status = self.create_publisher(
            Int32, f"{out_base}/output/solve_status", TEMPORAL_MPT_DEBUG_QOS
        )
        self._pub_accel = self.create_publisher(
            Float64MultiArray,
            f"{out_base}/output/control_acceleration_mps2",
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self._pub_delta = self.create_publisher(
            Float64MultiArray, f"{out_base}/output/control_delta_cmd_rad", TEMPORAL_MPT_DEBUG_QOS
        )

        self.create_subscription(
            Trajectory,
            f"{prefix}/input/reference_trajectory",
            self._on_reference,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Odometry, f"{prefix}/input/initial_state", self._on_odom, TEMPORAL_MPT_DEBUG_QOS
        )

        # rclpy has no warning_throttle (that is rclcpp); throttle manually
        self._last_no_odom_warn_mono = 0.0

        self.get_logger().info(f"Publishing Python MPC replay under {out_base}/output/...")

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_reference(self, msg: Trajectory) -> None:
        if self._last_odom is None:
            now = time.monotonic()
            if now - self._last_no_odom_warn_mono >= 2.0:
                self._last_no_odom_warn_mono = now
                self.get_logger().warning("No initial_state (odometry) yet; skipping solve.")
            return
        if len(msg.points) < 2:
            return

        odom = self._last_odom
        x0 = np.array(
            [
                float(odom.pose.pose.position.x),
                float(odom.pose.pose.position.y),
                _quaternion_to_yaw(odom.pose.pose.orientation),
                float(odom.twist.twist.linear.x),
            ]
        )

        n = len(msg.points)
        xs = np.array([float(p.pose.position.x) for p in msg.points])
        ys = np.array([float(p.pose.position.y) for p in msg.points])
        psis = np.array([_quaternion_to_yaw(p.pose.orientation) for p in msg.points])
        vs = np.array([float(p.longitudinal_velocity_mps) for p in msg.points])
        p_vec = np.array([self._lf, self._lr])

        try:
            NY_STAGE = 6
            sol_x, sol_u, status = self._solve_autoware(
                self._solver,
                self._N,
                x0,
                xs,
                ys,
                psis,
                vs,
                p_vec,
                NY_STAGE,
            )
        except Exception as exc:  # noqa: BLE001 — surface to log; keep node alive
            self.get_logger().error(f"solve_autoware_temporal_mpc failed: {exc}")
            st = Int32()
            st.data = -1
            self._pub_status.publish(st)
            return

        out_n = min(n, self._N + 1)
        out_traj = Trajectory()
        out_traj.header = msg.header
        if out_traj.header.frame_id == "":
            out_traj.header.frame_id = odom.header.frame_id if odom.header.frame_id else "map"
        out_traj.points = []
        for i in range(out_n):
            pt = TrajectoryPoint()
            row = sol_x[i]
            pt.pose.position.x = float(row[0])
            pt.pose.position.y = float(row[1])
            yaw = float(row[2])
            pt.pose.orientation.x = 0.0
            pt.pose.orientation.y = 0.0
            pt.pose.orientation.z = math.sin(yaw * 0.5)
            pt.pose.orientation.w = math.cos(yaw * 0.5)
            pt.longitudinal_velocity_mps = float(max(0.0, row[3]))
            uk = min(i, self._N - 1)
            pt.front_wheel_angle_rad = float(sol_u[uk][1])
            if i < self._N:
                pt.acceleration_mps2 = float(sol_u[i][0])
            out_traj.points.append(pt)

        if out_n >= 2:
            v0 = max(0.0, float(sol_x[0][3]))
            v1 = max(0.0, float(sol_x[1][3]))
            out_traj.points[0].longitudinal_velocity_mps = float(max(v0, v1))

        self._pub_traj.publish(out_traj)

        st = Int32()
        st.data = int(status)
        self._pub_status.publish(st)

        accel = Float64MultiArray()
        delta = Float64MultiArray()
        for k in range(self._N):
            accel.data.append(float(sol_u[k][0]))
            delta.data.append(float(sol_u[k][1]))
        self._pub_accel.publish(accel)
        self._pub_delta.publish(delta)


def main() -> None:
    rclpy.init()
    node = TemporalMptPythonReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
