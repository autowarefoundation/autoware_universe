#!/usr/bin/env python3

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

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from calc_utils import CalcUtils
import config as CF
from csv_reader import CSVReader
import matplotlib.pyplot as plt
import numpy as np
from plotter import Plotter
import rclpy
from rclpy.node import Node
import yaml


class DrawGraph(Node):
    calibrated_map_dir = ""

    def __init__(self):
        super().__init__("generic_value_map_plot_server")

        # Get default map directory
        self.declare_parameter("csv_default_map_dir", "")
        self.default_map_dir = (
            self.get_parameter("csv_default_map_dir").get_parameter_value().string_value
        )
        if not self.default_map_dir:
            default_map_path = get_package_share_directory("autoware_generic_value_converter")
            self.default_map_dir = default_map_path + "/data/"

        # Get calibrated map directory
        package_path = get_package_share_directory("autoware_generic_value_calibrator")
        self.declare_parameter("csv_calibrated_map_dir", package_path + "/config/")
        self.calibrated_map_dir = (
            self.get_parameter("csv_calibrated_map_dir").get_parameter_value().string_value
        )

        self.declare_parameter("calibration_method", "each_cell")
        self.calibration_method = (
            self.get_parameter("calibration_method").get_parameter_value().string_value
        )
        if self.calibration_method is None:
            self.calibration_method = "each_cell"
        elif not (
            (self.calibration_method == "each_cell") | (self.calibration_method == "four_cell")
        ):
            print("invalid method.")
            self.calibration_method = "each_cell"

        self.log_file = self.calibrated_map_dir + "log.csv"

        # Load config from parameter file
        config_file = package_path + "/config/generic_value_calibrator.param.yaml"
        if Path(config_file).exists():
            self.get_logger().info("config file exists")
            with open(config_file) as yml:
                data = yaml.safe_load(yml)
            params = data["/**"]["ros__parameters"]
            self.min_vel_thr = params.get("velocity_min_threshold", 0.1)
            self.vel_diff_thr = params.get("velocity_diff_threshold", 0.556)
            self.value_diff_thr = params.get("value_diff_threshold", 0.03)
            self.max_steer_thr = params.get("max_steer_threshold", 0.2)
            self.max_pitch_thr = params.get("max_pitch_threshold", 0.02)
            self.max_jerk_thr = params.get("max_jerk_threshold", 0.7)
        else:
            self.get_logger().warning("config file is not found in {}".format(config_file))
            self.min_vel_thr = 0.1
            self.vel_diff_thr = 0.556
            self.value_diff_thr = 0.03
            self.max_steer_thr = 0.2
            self.max_pitch_thr = 0.02
            self.max_jerk_thr = 0.7

        self.max_value_vel_thr = 0.7

        # Load indices from calibrated map or default map
        calibrated_map_file = self.calibrated_map_dir + "generic_value_map.csv"
        default_map_file = self.default_map_dir + "default_value_map.csv"

        if Path(calibrated_map_file).exists():
            CF.VALUE_LIST, CF.VEL_LIST = CF.load_indices_from_csv(calibrated_map_file)
            self.get_logger().info("Loaded indices from calibrated map")
        elif Path(default_map_file).exists():
            CF.VALUE_LIST, CF.VEL_LIST = CF.load_indices_from_csv(default_map_file)
            self.get_logger().info("Loaded indices from default map")
        else:
            CF.VALUE_LIST, CF.VEL_LIST = CF.get_default_indices()
            self.get_logger().warning("No map file found, using default indices")

        # Debug output
        self.get_logger().info("default map dir: {}".format(self.default_map_dir))
        self.get_logger().info("calibrated map dir: {}".format(self.calibrated_map_dir))
        self.get_logger().info("calibrated method: {}".format(self.calibration_method))
        self.get_logger().info("log file :{}".format(self.log_file))
        self.get_logger().info(
            "Value range: [{:.3f}, {:.3f}] with {} points".format(
                CF.VALUE_LIST.min(), CF.VALUE_LIST.max(), len(CF.VALUE_LIST)
            )
        )
        self.get_logger().info(
            "Velocity range: [{:.3f}, {:.3f}] m/s with {} points".format(
                CF.VEL_LIST.min(), CF.VEL_LIST.max(), len(CF.VEL_LIST)
            )
        )
        self.get_logger().info("min_vel_thr : {}".format(self.min_vel_thr))
        self.get_logger().info("vel_diff_thr : {}".format(self.vel_diff_thr))
        self.get_logger().info("value_diff_thr : {}".format(self.value_diff_thr))
        self.get_logger().info("max_steer_thr : {}".format(self.max_steer_thr))
        self.get_logger().info("max_pitch_thr : {}".format(self.max_pitch_thr))
        self.get_logger().info("max_jerk_thr : {}".format(self.max_jerk_thr))
        self.get_logger().info("max_value_vel_thr : {}".format(self.max_value_vel_thr))

        # Create timer to generate plots periodically
        self.create_timer(5.0, self.timer_callback)  # Update every 5 seconds

    def timer_callback(self):
        """Periodically generate and save visualization plots."""
        # If log file doesn't exist, skip
        if not Path(self.log_file).exists():
            return

        try:
            self.generate_plots()
            self.get_logger().info("Plots generated successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to generate plots: {e}")

    def generate_plots(self):
        """Generate visualization plots from log data."""
        # Read CSV
        self.cr = CSVReader(self.log_file, csv_type="file")

        # Remove unused data
        self.csv_data = self.cr.removeUnusedData(
            self.min_vel_thr,
            self.max_steer_thr,
            self.max_pitch_thr,
            self.max_value_vel_thr,
            self.max_jerk_thr,
        )

        if len(self.csv_data) == 0:
            self.get_logger().warning("No valid data after filtering")
            return

        # Get statistics array
        vel_data = self.cr.getVelData()
        value_data = self.cr.getValueData()
        acc_data = self.cr.getAccData()

        # Get color factor (pitch) array for plotting
        color_data = self.cr.getPitchData()

        data, full_data = CalcUtils.create_2d_map(
            vel_data,
            value_data,
            acc_data,
            color_data,
            CF.VEL_LIST / 3.6,
            self.vel_diff_thr,
            CF.VALUE_LIST,
            self.value_diff_thr,
            self.calibration_method,
        )

        count_map, average_map, stddev_map = CalcUtils.create_stat_map(data)
        velocity_map_list = []
        for i in range(len(CF.VEL_LIST)):
            velocity_map_list.append(CalcUtils.extract_x_index_map(full_data, i))

        default_value_list, default_acc_list = self.load_map(self.default_map_dir)
        if len(default_value_list) == 0 or len(default_acc_list) == 0:
            self.get_logger().warning(
                "No default map file was found in {}".format(self.default_map_dir)
            )
            return

        calibrated_value_list, calibrated_acc_list = self.load_map(self.calibrated_map_dir)

        # Visualize point from data
        plot_width = 3
        plot_height = int(math.ceil(len(CF.VEL_LIST) / float(plot_width)))
        plotter = Plotter(plot_height, plot_width)

        for i in range(len(CF.VEL_LIST)):
            self.view_value_accel_graph(
                plotter,
                i,
                velocity_map_list,
                i,
                count_map,
                average_map,
                stddev_map,
                default_value_list,
                default_acc_list,
                calibrated_value_list,
                calibrated_acc_list,
            )

        output_dir = Path(self.calibrated_map_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = output_dir / "plot.svg"
        plt.savefig(str(output_file))
        self.get_logger().info(f"svg saved to {output_file}")
        plt.close()

    def view_value_accel_graph(
        self,
        plotter,
        subplot_num,
        velocity_map_list,
        vel_list_idx,
        count_map,
        average_map,
        stddev_map,
        default_value_list,
        default_acc_list,
        calibrated_value_list,
        calibrated_acc_list,
    ):
        fig = plotter.subplot_more(subplot_num)

        # Calibrated map
        if len(calibrated_value_list) != 0 and len(calibrated_acc_list) != 0:
            if vel_list_idx < len(calibrated_acc_list):
                plotter.plot(
                    calibrated_value_list,
                    calibrated_acc_list[vel_list_idx],
                    color="blue",
                    label="calibrated",
                )

        # Default map
        if len(default_value_list) != 0 and len(default_acc_list) != 0:
            if vel_list_idx < len(default_acc_list):
                plotter.plot(
                    default_value_list,
                    default_acc_list[vel_list_idx],
                    color="orange",
                    label="default",
                    linestyle="dashed",
                )

        # Plot all data
        value_list = [0 for i in range(len(CF.VALUE_LIST))]
        if velocity_map_list[vel_list_idx] is not None:
            plotter.scatter_color(
                velocity_map_list[vel_list_idx][:, 1],
                velocity_map_list[vel_list_idx][:, 2],
                color=velocity_map_list[vel_list_idx][:, 3],
                label="all",
            )

            for value in velocity_map_list[vel_list_idx][:, 1]:
                min_value = 10
                for value_idx, ref_value in enumerate(CF.VALUE_LIST):
                    if min_value > abs(value - ref_value):
                        min_value = abs(value - ref_value)
                        min_value_idx = value_idx
                value_list[min_value_idx] += 1

        # Plot average data
        plotter.scatter(CF.VALUE_LIST, average_map[:, vel_list_idx], "red", label="average")

        # Add label of standard deviation
        plotter.scatter([], [], "black", label="std dev")

        # Plot average text
        for i in range(len(CF.VALUE_LIST)):
            if count_map[i, vel_list_idx] == 0:
                continue
            x = CF.VALUE_LIST[i]
            y = average_map[i, vel_list_idx]
            y2 = stddev_map[i, vel_list_idx]
            # Plot average
            plotter.plot_text(x, y + 1, y, color="red")

            # Plot standard deviation
            plotter.plot_text(x, y - 1, y2, color="black")

            # Plot the number of all data
            plotter.plot_text(
                x, y - 2, "{}\npts".format(value_list[i]), num_data_type="str", color="green"
            )

        value_lim = [CF.VALUE_LIST[0] - 0.05, CF.VALUE_LIST[-1] + 0.05]
        accel_lim = [-5.0, 5.0]

        plotter.set_lim(fig, value_lim, accel_lim)
        plotter.add_label(
            str(CF.VEL_LIST[vel_list_idx]) + "kmh; value-accel relation", "value", "accel"
        )

    def load_map(self, csv_dir):
        try:
            value_list = []
            acc_list = []
            csv_file = Path(csv_dir) / "generic_value_map.csv"

            if not csv_file.exists():
                return [], []

            with open(csv_file) as f:
                for l_idx, l in enumerate(f.readlines()):
                    w = l.split(",")
                    w[-1] = w[-1].strip()
                    if l_idx == 0:
                        # First line is velocity index
                        continue
                    elif l_idx == 1:
                        # Get value index from first column
                        value_list = [float(w[0])]
                    else:
                        value_list.append(float(w[0]))

                    # Store acceleration values for each velocity
                    if l_idx > 0:
                        acc_list.append([float(e) for e in w[1:]])

            return np.array(value_list), np.array(acc_list)
        except Exception as e:
            print(f"Error loading map: {e}")
            return [], []


def main(args=None):
    rclpy.init(args=args)
    node = DrawGraph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
