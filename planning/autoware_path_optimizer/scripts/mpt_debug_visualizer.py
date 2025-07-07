#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

from autoware_planning_msgs.msg import Trajectory
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node


class MPTDebugVisualizer(Node):
    def __init__(self):
        super().__init__("mpt_debug_visualizer")

        # Subscribers
        self.sub_mpt_traj = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/mpt_traj",
            self.plotMPTTrajectory,
            1,
        )
        self.sub_mpt_steering = self.create_subscription(
            Float32MultiArray,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/optimised_steering",
            self.plotMPTSteering,
            1,
        )
        self.sub_acados_traj = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/acados_mpt_traj",
            self.plotAcadosTrajectory,
            1,
        )
        self.sub_acados_steering = self.create_subscription(
            Float32MultiArray,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/acados_optimised_steering",
            self.plotAcadosSteering,
            1,
        )
        self.sub_mpt_states = self.create_subscription(
            Float32MultiArray,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/optimised_states",
            self.plotMPTStates,
            1,
        )
        self.sub_acados_states = self.create_subscription(
            Float32MultiArray,
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/acados_optimised_states",
            self.plotAcadosStates,
            1,
        )

        # Initialize plots - use non-blocking mode to prevent focus stealing
        plt.ion()
        self.fig = plt.figure(figsize=(16, 14))
        
        # Create subplots: 3x2 layout
        # Row 1: trajectory plots (x-y)
        self.ax_traj_mpt = self.fig.add_subplot(3, 2, 1)
        self.ax_traj_mpt.set_title("MPT Trajectory (x-y)")
        self.ax_traj_mpt.set_xlabel("x [m]")
        self.ax_traj_mpt.set_ylabel("y [m]")
        self.ax_traj_mpt.grid(True)
        self.ax_traj_mpt.set_aspect('equal')
        self.line_traj_mpt, = self.ax_traj_mpt.plot([], [], 'b-', label='MPT Trajectory', linewidth=2)
        self.ax_traj_mpt.legend()

        self.ax_traj_acados = self.fig.add_subplot(3, 2, 2)
        self.ax_traj_acados.set_title("Acados MPT Trajectory (x-y)")
        self.ax_traj_acados.set_xlabel("x [m]")
        self.ax_traj_acados.set_ylabel("y [m]")
        self.ax_traj_acados.grid(True)
        self.ax_traj_acados.set_aspect('equal')
        self.line_traj_acados, = self.ax_traj_acados.plot([], [], 'r-', label='Acados Trajectory', linewidth=2)
        self.ax_traj_acados.legend()

        # Row 2: steering angle plots
        self.ax_steering_mpt = self.fig.add_subplot(3, 2, 3)
        self.ax_steering_mpt.set_title("MPT Optimized Steering Angles")
        self.ax_steering_mpt.set_xlabel("Arc Length [m]")
        self.ax_steering_mpt.set_ylabel("Steering Angle [rad]")
        self.ax_steering_mpt.grid(True)
        self.line_steering_mpt, = self.ax_steering_mpt.plot([], [], 'b-o', label='MPT Steering', markersize=4)
        self.ax_steering_mpt.legend()

        self.ax_steering_acados = self.fig.add_subplot(3, 2, 4)
        self.ax_steering_acados.set_title("Acados Optimized Steering Angles")
        self.ax_steering_acados.set_xlabel("Arc Length [m]")
        self.ax_steering_acados.set_ylabel("Steering Angle [rad]")
        self.ax_steering_acados.grid(True)
        self.line_steering_acados, = self.ax_steering_acados.plot([], [], 'r-o', label='Acados Steering', markersize=4)
        self.ax_steering_acados.legend()

        # Row 3: state plots (eY and ePsi - shared plots for comparison)
        self.ax_ey = self.fig.add_subplot(3, 2, 5)
        self.ax_ey.set_title("Lateral Error (eY)")
        self.ax_ey.set_xlabel("Point Index")
        self.ax_ey.set_ylabel("eY [m]")
        self.ax_ey.grid(True)
        self.line_ey_mpt, = self.ax_ey.plot([], [], 'b-o', label='MPT eY', markersize=4)
        self.line_ey_acados, = self.ax_ey.plot([], [], 'r-o', label='Acados eY', markersize=4)
        self.ax_ey.legend()

        self.ax_epsi = self.fig.add_subplot(3, 2, 6)
        self.ax_epsi.set_title("Yaw Error (ePsi)")
        self.ax_epsi.set_xlabel("Point Index")
        self.ax_epsi.set_ylabel("ePsi [rad]")
        self.ax_epsi.grid(True)
        self.line_epsi_mpt, = self.ax_epsi.plot([], [], 'b-s', label='MPT ePsi', markersize=4)
        self.line_epsi_acados, = self.ax_epsi.plot([], [], 'r-s', label='Acados ePsi', markersize=4)
        self.ax_epsi.legend()

        plt.tight_layout()
        plt.show(block=False)  # Non-blocking to prevent focus stealing

        # Store latest data for comparison plot
        self.latest_mpt_traj = None
        self.latest_acados_traj = None
        self.latest_mpt_steering = None
        self.latest_acados_steering = None
        self.latest_mpt_states = None
        self.latest_acados_states = None

    def calcArcLength(self, traj):
        """Calculate cumulative arc length along trajectory."""
        if len(traj.points) == 0:
            return []
        
        s_arr = [0.0]
        s_sum = 0.0
        
        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        
        return s_arr

    def plotMPTTrajectory(self, msg):
        """Plot MPT trajectory in x-y space."""
        if len(msg.points) == 0:
            return
        
        x = [p.pose.position.x for p in msg.points]
        y = [p.pose.position.y for p in msg.points]
        
        self.line_traj_mpt.set_data(x, y)
        self.ax_traj_mpt.relim()
        self.ax_traj_mpt.autoscale_view(True, True, True)
        
        self.latest_mpt_traj = msg
        self.updatePlot()

    def plotAcadosTrajectory(self, msg):
        """Plot Acados MPT trajectory in x-y space."""
        if len(msg.points) == 0:
            return
        
        x = [p.pose.position.x for p in msg.points]
        y = [p.pose.position.y for p in msg.points]
        
        self.line_traj_acados.set_data(x, y)
        self.ax_traj_acados.relim()
        self.ax_traj_acados.autoscale_view(True, True, True)
        
        self.latest_acados_traj = msg
        self.updatePlot()

    def plotMPTSteering(self, msg):
        """Plot MPT optimized steering angles."""
        if len(msg.data) == 0:
            return
        
        steering_angles = list(msg.data)
        
        # If we have a trajectory, use its arc length
        if self.latest_mpt_traj is not None and len(self.latest_mpt_traj.points) > 0:
            s = self.calcArcLength(self.latest_mpt_traj)
            # Interpolate or match steering to trajectory points
            if len(s) == len(steering_angles):
                self.line_steering_mpt.set_data(s, steering_angles)
            elif len(s) > 0:
                # If sizes don't match, create evenly spaced arc length
                s_uniform = np.linspace(0, s[-1], len(steering_angles))
                self.line_steering_mpt.set_data(s_uniform, steering_angles)
            else:
                s_uniform = np.arange(len(steering_angles))
                self.line_steering_mpt.set_data(s_uniform, steering_angles)
        else:
            # No trajectory yet, use index as x-axis
            s_uniform = np.arange(len(steering_angles))
            self.line_steering_mpt.set_data(s_uniform, steering_angles)
        
        self.ax_steering_mpt.relim()
        self.ax_steering_mpt.autoscale_view(True, True, True)
        
        self.latest_mpt_steering = steering_angles
        self.updatePlot()

    def plotAcadosSteering(self, msg):
        """Plot Acados optimized steering angles."""
        if len(msg.data) == 0:
            return
        
        steering_angles = list(msg.data)
        
        # If we have a trajectory, use its arc length
        if self.latest_acados_traj is not None and len(self.latest_acados_traj.points) > 0:
            s = self.calcArcLength(self.latest_acados_traj)
            # Interpolate or match steering to trajectory points
            if len(s) == len(steering_angles):
                self.line_steering_acados.set_data(s, steering_angles)
            elif len(s) > 0:
                # If sizes don't match, create evenly spaced arc length
                s_uniform = np.linspace(0, s[-1], len(steering_angles))
                self.line_steering_acados.set_data(s_uniform, steering_angles)
            else:
                s_uniform = np.arange(len(steering_angles))
                self.line_steering_acados.set_data(s_uniform, steering_angles)
        else:
            # No trajectory yet, use index as x-axis
            s_uniform = np.arange(len(steering_angles))
            self.line_steering_acados.set_data(s_uniform, steering_angles)
        
        self.ax_steering_acados.relim()
        self.ax_steering_acados.autoscale_view(True, True, True)
        
        self.latest_acados_steering = steering_angles
        self.updatePlot()

    def parseStates(self, msg):
        """Parse state message: [eY_0, ePsi_0, eY_1, ePsi_1, ...]"""
        if len(msg.data) == 0 or len(msg.data) % 2 != 0:
            return [], []
        
        eY = []
        ePsi = []
        for i in range(0, len(msg.data), 2):
            eY.append(msg.data[i])
            ePsi.append(msg.data[i + 1])
        
        return eY, ePsi

    def plotMPTStates(self, msg):
        """Plot MPT optimized states (eY and ePsi)."""
        if len(msg.data) == 0:
            return
        
        eY, ePsi = self.parseStates(msg)
        if len(eY) == 0:
            return
        
        indices = np.arange(len(eY))
        
        self.line_ey_mpt.set_data(indices, eY)
        self.line_epsi_mpt.set_data(indices, ePsi)
        
        # Update both axes to accommodate both MPT and Acados data
        self.ax_ey.relim()
        self.ax_ey.autoscale_view(True, True, True)
        self.ax_epsi.relim()
        self.ax_epsi.autoscale_view(True, True, True)
        
        self.latest_mpt_states = {'eY': eY, 'ePsi': ePsi}
        self.updatePlot()

    def plotAcadosStates(self, msg):
        """Plot Acados optimized states (eY and ePsi)."""
        if len(msg.data) == 0:
            return
        
        eY, ePsi = self.parseStates(msg)
        if len(eY) == 0:
            return
        
        indices = np.arange(len(eY))
        
        self.line_ey_acados.set_data(indices, eY)
        self.line_epsi_acados.set_data(indices, ePsi)
        
        # Update both axes to accommodate both MPT and Acados data
        self.ax_ey.relim()
        self.ax_ey.autoscale_view(True, True, True)
        self.ax_epsi.relim()
        self.ax_epsi.autoscale_view(True, True, True)
        
        self.latest_acados_states = {'eY': eY, 'ePsi': ePsi}
        self.updatePlot()

    def updatePlot(self):
        """Update the plot display."""
        self.fig.canvas.draw_idle()  # Use draw_idle instead of draw to avoid blocking
        self.fig.canvas.flush_events()  # Process events without blocking


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MPTDebugVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
