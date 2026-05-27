# Copyright 2026 TIER IV, Inc.
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

import argparse
from dataclasses import dataclass
from dataclasses import field
import math
import sys
import threading
from typing import List
from typing import Optional
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
import matplotlib
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32


def trajectory_to_xy(points) -> Tuple[List[float], List[float]]:
    xs = [p.pose.position.x for p in points]
    ys = [p.pose.position.y for p in points]
    return xs, ys


def trajectory_velocity_steering(
    points, n_cap: Optional[int] = None
) -> Tuple[List[float], List[float]]:
    n = len(points) if n_cap is None else min(len(points), n_cap)
    vs = [float(p.longitudinal_velocity_mps) for p in points[:n]]
    deltas = [float(p.front_wheel_angle_rad) for p in points[:n]]
    return vs, deltas


@dataclass
class DebugFrame:
    input_xy: Optional[Tuple[List[float], List[float]]] = None
    output_xy: Optional[Tuple[List[float], List[float]]] = None
    input_vel: List[float] = field(default_factory=list)
    input_delta: List[float] = field(default_factory=list)
    output_vel: List[float] = field(default_factory=list)
    output_delta: List[float] = field(default_factory=list)
    control_accel: List[float] = field(default_factory=list)
    control_delta_cmd: List[float] = field(default_factory=list)
    initial_xy_yaw: Optional[Tuple[float, float, float]] = None
    solve_status: Optional[int] = None
    # Optional: temporal_mpt_python_reference.py replay (same inputs as C++ debug I/O)
    python_output_xy: Optional[Tuple[List[float], List[float]]] = None
    python_output_vel: List[float] = field(default_factory=list)
    python_output_delta: List[float] = field(default_factory=list)
    python_control_accel: List[float] = field(default_factory=list)
    python_control_delta_cmd: List[float] = field(default_factory=list)
    python_solve_status: Optional[int] = None


class TemporalMptDebugVisualizer(Node):
    def __init__(self, *, topic_prefix: str, update_hz: float, compare_python: bool) -> None:
        super().__init__("temporal_mpt_debug_visualizer")

        update_hz = max(update_hz, 1.0)

        self._lock = threading.Lock()
        self._frame = DebugFrame()
        self._logged_first_reference = False

        # Publishers use best_effort (see trajectory_temporal_mpt_optimizer.cpp). rclpy default is reliable.
        TEMPORAL_MPT_DEBUG_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Trajectory,
            f"{topic_prefix}/input/reference_trajectory",
            self.on_input_trajectory,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Trajectory,
            f"{topic_prefix}/output/trajectory",
            self.on_output_trajectory,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Odometry,
            f"{topic_prefix}/input/initial_state",
            self.on_initial_state,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Int32,
            f"{topic_prefix}/output/solve_status",
            self.on_solve_status,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            f"{topic_prefix}/output/control_acceleration_mps2",
            self.on_control_accel,
            TEMPORAL_MPT_DEBUG_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            f"{topic_prefix}/output/control_delta_cmd_rad",
            self.on_control_delta_cmd,
            TEMPORAL_MPT_DEBUG_QOS,
        )

        self._compare_python = compare_python
        if compare_python:
            py = f"{topic_prefix}/python"
            self.create_subscription(
                Trajectory,
                f"{py}/output/trajectory",
                self.on_python_output_trajectory,
                TEMPORAL_MPT_DEBUG_QOS,
            )
            self.create_subscription(
                Int32,
                f"{py}/output/solve_status",
                self.on_python_solve_status,
                TEMPORAL_MPT_DEBUG_QOS,
            )
            self.create_subscription(
                Float64MultiArray,
                f"{py}/output/control_acceleration_mps2",
                self.on_python_control_accel,
                TEMPORAL_MPT_DEBUG_QOS,
            )
            self.create_subscription(
                Float64MultiArray,
                f"{py}/output/control_delta_cmd_rad",
                self.on_python_control_delta_cmd,
                TEMPORAL_MPT_DEBUG_QOS,
            )

        self._fig = plt.figure(figsize=(14, 8))
        gs = gridspec.GridSpec(
            3, 2, figure=self._fig, width_ratios=[1.15, 1.0], wspace=0.28, hspace=0.35
        )
        self._ax_xy = self._fig.add_subplot(gs[:, 0])
        self._ax_v = self._fig.add_subplot(gs[0, 1])
        self._ax_delta = self._fig.add_subplot(gs[1, 1])
        self._ax_u = self._fig.add_subplot(gs[2, 1])
        self._ax_u_twin = self._ax_u.twinx()

        self._fig.canvas.manager.set_window_title("Temporal MPT Debug Visualizer")
        self._configure_window_no_focus_steal()
        plt.show(block=False)

        self.get_logger().info("Temporal MPT debug visualizer started.")
        self.get_logger().info(f"Listening under prefix: {topic_prefix}")
        if compare_python:
            self.get_logger().info(f"Python comparison enabled: {topic_prefix}/python/output/...")
        self.get_logger().info(
            "Subscriptions use BEST_EFFORT QoS (must match temporal MPT debug publishers)."
        )

        timer_period = 1.0 / update_hz
        self.create_timer(timer_period, self.on_timer)

    def _configure_window_no_focus_steal(self) -> None:
        """Best-effort hints so redraws do not activate the plot window (Linux Tk/Qt)."""
        try:
            win = self._fig.canvas.manager.window
        except (AttributeError, TypeError):
            return
        if win is None:
            return
        if hasattr(win, "attributes"):
            try:
                win.attributes("-topmost", False)
            except Exception:
                pass
        try:
            from PyQt5 import QtCore  # type: ignore[import-not-found]

            win.setAttribute(QtCore.Qt.WA_ShowWithoutActivating, True)
        except (ImportError, AttributeError):
            pass

    def on_input_trajectory(self, msg: Trajectory) -> None:
        with self._lock:
            self._frame.input_xy = trajectory_to_xy(msg.points)
            self._frame.input_vel, self._frame.input_delta = trajectory_velocity_steering(
                msg.points
            )
        if not self._logged_first_reference and len(msg.points) > 0:
            self._logged_first_reference = True
            self.get_logger().info(f"Receiving reference_trajectory ({len(msg.points)} points).")

    def on_output_trajectory(self, msg: Trajectory) -> None:
        with self._lock:
            self._frame.output_xy = trajectory_to_xy(msg.points)
            self._frame.output_vel, self._frame.output_delta = trajectory_velocity_steering(
                msg.points
            )

    def on_initial_state(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._frame.initial_xy_yaw = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
            )

    def on_solve_status(self, msg: Int32) -> None:
        with self._lock:
            self._frame.solve_status = msg.data

    def on_control_accel(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._frame.control_accel = list(msg.data)

    def on_control_delta_cmd(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._frame.control_delta_cmd = list(msg.data)

    def on_python_output_trajectory(self, msg: Trajectory) -> None:
        with self._lock:
            self._frame.python_output_xy = trajectory_to_xy(msg.points)
            self._frame.python_output_vel, self._frame.python_output_delta = (
                trajectory_velocity_steering(msg.points)
            )

    def on_python_solve_status(self, msg: Int32) -> None:
        with self._lock:
            self._frame.python_solve_status = msg.data

    def on_python_control_accel(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._frame.python_control_accel = list(msg.data)

    def on_python_control_delta_cmd(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._frame.python_control_delta_cmd = list(msg.data)

    def on_timer(self) -> None:
        with self._lock:
            frame = DebugFrame(
                input_xy=self._frame.input_xy,
                output_xy=self._frame.output_xy,
                input_vel=list(self._frame.input_vel),
                input_delta=list(self._frame.input_delta),
                output_vel=list(self._frame.output_vel),
                output_delta=list(self._frame.output_delta),
                control_accel=list(self._frame.control_accel),
                control_delta_cmd=list(self._frame.control_delta_cmd),
                initial_xy_yaw=self._frame.initial_xy_yaw,
                solve_status=self._frame.solve_status,
                python_output_xy=self._frame.python_output_xy,
                python_output_vel=list(self._frame.python_output_vel),
                python_output_delta=list(self._frame.python_output_delta),
                python_control_accel=list(self._frame.python_control_accel),
                python_control_delta_cmd=list(self._frame.python_control_delta_cmd),
                python_solve_status=self._frame.python_solve_status,
            )

        # --- Left: XY path ---
        self._ax_xy.clear()
        self._ax_xy.set_title("Temporal MPT path (map)")
        self._ax_xy.set_xlabel("x [m]")
        self._ax_xy.set_ylabel("y [m]")
        self._ax_xy.grid(True)

        if frame.input_xy is not None and len(frame.input_xy[0]) > 0:
            self._ax_xy.plot(
                frame.input_xy[0],
                frame.input_xy[1],
                "bo",
                markersize=4,
                linestyle="",
                label="input reference",
            )
        if frame.output_xy is not None and len(frame.output_xy[0]) > 0:
            self._ax_xy.plot(
                frame.output_xy[0],
                frame.output_xy[1],
                "r-",
                linewidth=2,
                label="output (C++)",
            )
        if frame.python_output_xy is not None and len(frame.python_output_xy[0]) > 0:
            self._ax_xy.plot(
                frame.python_output_xy[0],
                frame.python_output_xy[1],
                color="darkorange",
                linestyle="--",
                linewidth=2,
                label="output (Python)",
            )

        if frame.initial_xy_yaw is not None:
            x0, y0, yaw = frame.initial_xy_yaw
            self._ax_xy.plot([x0], [y0], "ko", label="initial state")
            self._ax_xy.arrow(
                x0,
                y0,
                2.0 * math.cos(yaw),
                2.0 * math.sin(yaw),
                head_width=0.4,
                head_length=0.6,
                fc="k",
                ec="k",
            )

        if frame.solve_status is not None or frame.python_solve_status is not None:
            parts = []
            if frame.solve_status is not None:
                parts.append(f"C++ status: {frame.solve_status}")
            if frame.python_solve_status is not None:
                parts.append(f"Py status: {frame.python_solve_status}")
            status_text = " | ".join(parts)
            ok_cpp = frame.solve_status is None or frame.solve_status == 0
            ok_py = frame.python_solve_status is None or frame.python_solve_status == 0
            status_color = "green" if (ok_cpp and ok_py) else "red"
            self._ax_xy.text(
                0.02,
                0.98,
                status_text,
                transform=self._ax_xy.transAxes,
                verticalalignment="top",
                color=status_color,
                fontsize=10,
                bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": status_color},
            )

        if (
            (frame.input_xy and len(frame.input_xy[0]) > 0)
            or (frame.output_xy and len(frame.output_xy[0]) > 0)
            or (frame.python_output_xy and len(frame.python_output_xy[0]) > 0)
            or frame.initial_xy_yaw is not None
        ):
            self._ax_xy.relim()
            self._ax_xy.autoscale_view()
        self._ax_xy.set_aspect("equal", adjustable="datalim")
        self._ax_xy.legend(loc="best")

        # --- Right top: longitudinal velocity (reference vs optimized state) ---
        self._ax_v.clear()
        self._ax_v.set_title("Velocity (reference vs optimized state)")
        self._ax_v.set_xlabel("point index")
        self._ax_v.set_ylabel("v [m/s]")
        self._ax_v.grid(True)
        n_compare = 0
        if frame.input_vel and frame.output_vel:
            n_compare = min(len(frame.input_vel), len(frame.output_vel))
        if n_compare > 0:
            idx = list(range(n_compare))
            self._ax_v.plot(
                idx, frame.input_vel[:n_compare], "b.--", markersize=5, label="input ref v"
            )
            self._ax_v.plot(
                idx, frame.output_vel[:n_compare], "r-", linewidth=2, label="output v (C++)"
            )
            if frame.python_output_vel:
                n_py = min(n_compare, len(frame.python_output_vel))
                self._ax_v.plot(
                    list(range(n_py)),
                    frame.python_output_vel[:n_py],
                    color="darkorange",
                    linestyle="--",
                    linewidth=2,
                    label="output v (Python)",
                )
            self._ax_v.legend(loc="best")
        elif frame.input_vel:
            self._ax_v.plot(
                range(len(frame.input_vel)),
                frame.input_vel,
                "b.--",
                markersize=5,
                label="input ref v",
            )
            self._ax_v.legend(loc="best")
        elif frame.output_vel:
            self._ax_v.plot(
                range(len(frame.output_vel)),
                frame.output_vel,
                "r-",
                linewidth=2,
                label="output state v",
            )
            self._ax_v.legend(loc="best")

        # --- Right middle: steering angle δ state (reference vs optimized) ---
        self._ax_delta.clear()
        self._ax_delta.set_title("Steering δ (reference vs optimized state)")
        self._ax_delta.set_xlabel("point index")
        self._ax_delta.set_ylabel("δ [rad]")
        self._ax_delta.grid(True)
        if n_compare > 0:
            idx = list(range(n_compare))
            self._ax_delta.plot(
                idx, frame.input_delta[:n_compare], "b.--", markersize=5, label="input ref δ"
            )
            self._ax_delta.plot(
                idx, frame.output_delta[:n_compare], "r-", linewidth=2, label="output δ (C++)"
            )
            if frame.python_output_delta:
                n_py = min(n_compare, len(frame.python_output_delta))
                self._ax_delta.plot(
                    list(range(n_py)),
                    frame.python_output_delta[:n_py],
                    color="darkorange",
                    linestyle="--",
                    linewidth=2,
                    label="output δ (Python)",
                )
            self._ax_delta.legend(loc="best")
        elif frame.input_delta:
            self._ax_delta.plot(
                range(len(frame.input_delta)),
                frame.input_delta,
                "b.--",
                markersize=5,
                label="input ref δ",
            )
            self._ax_delta.legend(loc="best")
        elif frame.output_delta:
            self._ax_delta.plot(
                range(len(frame.output_delta)),
                frame.output_delta,
                "r-",
                linewidth=2,
                label="output state δ",
            )
            self._ax_delta.legend(loc="best")

        # --- Right bottom: MPC controls a and δ_cmd vs stage ---
        self._ax_u.clear()
        self._ax_u_twin.clear()

        self._ax_u.set_title("MPC controls (per stage)")
        self._ax_u.set_xlabel("stage k")
        self._ax_u.grid(True)

        na = len(frame.control_accel)
        nd = len(frame.control_delta_cmd)
        n_stages = min(na, nd) if na and nd else max(na, nd)
        if n_stages > 0:
            stages = list(range(n_stages))
            self._ax_u.set_ylabel("a [m/s²]", color="tab:green")
            if na >= n_stages:
                self._ax_u.plot(
                    stages, frame.control_accel[:n_stages], "g-", linewidth=1.5, label="a C++"
                )
            self._ax_u.tick_params(axis="y", labelcolor="tab:green")
            npa = len(frame.python_control_accel)
            if npa > 0:
                n_ap = min(n_stages, npa)
                self._ax_u.plot(
                    stages[:n_ap],
                    frame.python_control_accel[:n_ap],
                    color="darkgoldenrod",
                    linestyle="--",
                    linewidth=1.5,
                    label="a Python",
                )

            self._ax_u_twin.set_ylabel("δ_cmd [rad]", color="tab:purple")
            if nd >= n_stages:
                self._ax_u_twin.plot(
                    stages,
                    frame.control_delta_cmd[:n_stages],
                    color="tab:purple",
                    linestyle="-",
                    linewidth=1.5,
                    label="δ_cmd C++",
                )
            self._ax_u_twin.tick_params(axis="y", labelcolor="tab:purple")
            npd = len(frame.python_control_delta_cmd)
            if npd > 0:
                n_dp = min(n_stages, npd)
                self._ax_u_twin.plot(
                    stages[:n_dp],
                    frame.python_control_delta_cmd[:n_dp],
                    color="darkorange",
                    linestyle="--",
                    linewidth=1.5,
                    label="δ_cmd Python",
                )

            lines = self._ax_u.get_lines() + self._ax_u_twin.get_lines()
            labels = [ln.get_label() for ln in lines]
            self._ax_u.legend(lines, labels, loc="upper right", fontsize=8)
            if frame.solve_status is not None and frame.solve_status != 0:
                self._ax_u.text(
                    0.02,
                    0.02,
                    f"last SQP iterate (solve_status={frame.solve_status})",
                    transform=self._ax_u.transAxes,
                    fontsize=8,
                    color="darkorange",
                    verticalalignment="bottom",
                )
        else:
            self._ax_u.text(
                0.5,
                0.5,
                "no MPC control data",
                ha="center",
                va="center",
                transform=self._ax_u.transAxes,
            )

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


def parse_args(argv: list[str]) -> argparse.Namespace:
    default_prefix = "/planning/trajectory_generator/trajectory_optimizer_node/debug/temporal_mpt"
    parser = argparse.ArgumentParser(
        description="Plot temporal MPT debug: path, state (v, δ), and MPC controls.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--topic-prefix",
        default=default_prefix,
        help="Topic prefix (same as TrajectoryTemporalMPTOptimizer ~/debug/temporal_mpt/...)",
    )
    parser.add_argument(
        "--update-hz",
        type=float,
        default=10.0,
        help="Matplotlib refresh rate",
    )
    parser.add_argument(
        "--compare-python",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Also subscribe to <prefix>/python/output/... from temporal_mpt_python_reference.py",
    )
    return parser.parse_args(argv)


def main() -> None:
    filtered_argv = remove_ros_args(args=sys.argv)
    cli = parse_args(filtered_argv[1:])
    rclpy.init(args=filtered_argv)

    node = TemporalMptDebugVisualizer(
        topic_prefix=cli.topic_prefix.rstrip("/"),
        update_hz=cli.update_hz,
        compare_python=cli.compare_python,
    )

    # Do not bring the plot window to front on every redraw (stops stealing focus from other apps).
    try:
        matplotlib.rcParams["figure.raise_window"] = False
    except KeyError:
        pass

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not plt.fignum_exists(node._fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
