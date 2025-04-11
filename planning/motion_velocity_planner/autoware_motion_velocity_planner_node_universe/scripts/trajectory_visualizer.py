#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 TIER IV, Inc. All rights reserved.
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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()

        self.min_vel = -1.0
        self.max_vel = 0.0
        self.min_s = -1.0
        self.max_s = 0.0

        # update flag
        self.update_input_traj = False
        self.update_output_traj = False
        self.update_run_out_traj = False
        self.update_mvp_traj = False

        self.self_pose = Pose()
        self.self_pose_received = False
        self.localization_twist = Twist()

        self.setPlotTrajectoryVelocity()

        self.input_trajectory = Trajectory()
        self.output_trajectory = Trajectory()
        self.run_out_trajectory = Trajectory()
        self.mvp_trajectory = Trajectory()

        self.sub_localization_twist = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.CallbackLocalizationOdom, 1
        )

        topic_header = "/planning/scenario_planning/lane_driving/motion_planning/"
        traj_input = "path_optimizer/trajectory"
        self.sub_input = self.create_subscription(
            Trajectory, topic_header + traj_input, self.CallbackInputTraj, 1
        )
        traj_output = "/planning/scenario_planning/trajectory"
        self.sub_output = self.create_subscription(
            Trajectory, traj_output, self.CallbackOutputTraj, 1
        )
        traj_run_out = "motion_velocity_planner/debug/run_out/trajectory"
        self.sub_run_out = self.create_subscription(
            Trajectory, topic_header + traj_run_out, self.CallbackRunOutTraj, 1
        )
        traj_mvp = "motion_velocity_planner/trajectory"
        self.sub_mvp = self.create_subscription(
            Trajectory, topic_header + traj_mvp, self.CallbackMVPTraj, 1
        )
        plt.show(block=False)

    def CallbackLocalizationOdom(self, cmd):
        self.self_pose = cmd.pose.pose
        self.localization_twist = cmd.twist.twist
        if self.current_velocity_plot:
            self.current_velocity_plot.set_data([0], [self.localization_twist.linear.x])

    def CallbackInputTraj(self, cmd):
        self.updatePlot(self.input_plot, cmd)

    def CallbackOutputTraj(self, cmd):
        self.updatePlot(self.output_plot, cmd)
        # Only redraw when getting the output trajectory
        self.ax1.set_xlim(self.min_s, self.max_s + 1.0)
        self.ax1.set_ylim(self.min_vel, self.max_vel + 1.0)
        self.max_s = self.max_vel = 0
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def CallbackRunOutTraj(self, cmd):
        self.updatePlot(self.run_out_plot, cmd)

    def CallbackMVPTraj(self, cmd):
        self.updatePlot(self.mvp_plot, cmd)

    def setPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(1, 1, 1)  # row, col, index(<raw*col)
        (self.input_plot,) = self.ax1.plot([], [], label="Input Velocity", ls=":", color="black")
        (self.output_plot,) = self.ax1.plot([], [], label="Output Velocity", ls=":", color="blue")
        (self.mvp_plot,) = self.ax1.plot(
            [], [], label="MVP (all modules) Velocity", ls="-", color="green"
        )
        (self.run_out_plot,) = self.ax1.plot(
            [], [], label="run_out Velocity", color="pink", ls="--", markersize=10
        )
        (self.current_velocity_plot,) = self.ax1.plot(
            [], [], label="Current velocity", color="red", marker="+", markersize=10
        )
        self.ax1.set_title("Motion Planning - Velocity Profiles")
        self.ax1.legend()
        self.ax1.set_ylabel("vel [m/s]")
        plt.autoscale(True)
        self.ax1.set_autoscale_on(True)
        self.ax1.margins(1.0, 1.0)

    def updatePlot(self, plot, trajectory):
        closest_idx = self.calcClosestTrajectory(trajectory)
        x = self.CalcArcLength(trajectory)
        x = [a - x[closest_idx] for a in x]
        self.max_s = max(max(x), self.max_s)
        y = self.ToVelList(trajectory)
        self.max_vel = max(max(y), self.max_vel)
        plot.set_data(x, y)

    def CalcArcLength(self, traj):
        return self.CalcPartArcLength(traj.points, 0, len(traj.points))

    def CalcPartArcLength(self, traj, start, end):
        assert start <= end
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj) > 0:
            s_arr.append(s_sum)

        for i in range(start + 1, end):
            p0 = traj[i - 1]
            p1 = traj[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def CalcTrajectoryInterval(self, traj, start, end):
        ds_arr = []

        for i in range(start + 1, end):
            p0 = traj[i - 1]
            p1 = traj[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            ds_arr.append(ds)
        return ds_arr

    def CalcTime(self, traj, start, end):
        t_arr = []
        t_sum = 0.0
        ds_arr = self.CalcTrajectoryInterval(traj, start, end)

        if len(traj) > 0:
            t_arr.append(t_sum)

        for i in range(start, end - 1):
            v = traj[i].longitudinal_velocity_mps
            ds = ds_arr[i - start]
            dt = ds / max(v, 0.1)
            t_sum += dt
            t_arr.append(t_sum)
        return t_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.longitudinal_velocity_mps)
        return v_list

    def calcClosestTrajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcSquaredDist2d(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return dx * dx + dy * dy


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrajectoryVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
