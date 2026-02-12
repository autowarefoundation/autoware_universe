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

import sys

import numpy as np
from scipy import signal

BIT = 1e-04


def get_map_list(y_num, x_num):
    data_list = []
    for yn in range(y_num):
        child_data_list = []
        for xn in range(x_num):
            child_data_list.append([])
            if xn == x_num - 1:
                data_list.append(child_data_list)
    return data_list


class CalcUtils:
    @staticmethod
    def average_filter(data, average_num):
        if not isinstance(average_num, int):
            print(
                "Error in average_filter(data, average_num):\
                Type of average_num must be int"
            )
            sys.exit(1)

        if average_num % 2 == 0:
            print(
                "Error in average_filter(data, average_num):\
                average_num must be odd number"
            )
            sys.exit(1)

        average_filter = np.ones(average_num) / float(average_num)
        average_data = np.convolve(data, average_filter)
        cut_num = int((average_num - 1) / 2)
        return average_data[cut_num : len(average_data) - cut_num]

    @staticmethod
    # fp:pass Hz, #fs: block Hz, g_pass: pass dB, g_stop: stop DB
    def lowpass_filter_scipy(x, sample_rate, fp, fs, g_pass, g_stop):
        fn = sample_rate / 2
        wp = fp / fn
        ws = fs / fn
        N, Wn = signal.buttord(wp, ws, g_pass, g_stop)
        b, a = signal.butter(N, Wn, "low")
        y = signal.filtfilt(b, a, x)
        return y

    @staticmethod
    def create_2d_map(
        x,
        y,
        data,
        color_factor,
        x_index_list,
        x_thresh,
        y_index_list,
        y_thresh,
        calibration_method="four_cell",
    ):
        if x.shape != y.shape or y.shape != data.shape:
            print("Error: the shape of x, y, data must be same")
            sys.exit()
        data_size = len(x)

        x_num = len(x_index_list)
        y_num = len(y_index_list)
        data_list = get_map_list(y_num, x_num)
        full_data_list = get_map_list(y_num, x_num)

        if calibration_method == "four_cell":
            x_thresh = np.abs(x_index_list[1] - x_index_list[0]) / 2
            y_thresh = np.abs(y_index_list[1] - y_index_list[0]) / 2

        for i in range(0, data_size):
            x_index = None
            y_index = None
            for xi in range(0, x_num):
                if np.abs(x_index_list[xi] - x[i]) < x_thresh:
                    x_index = xi
                    break

            for yi in range(0, y_num):
                if np.abs(y_index_list[yi] - y[i]) < y_thresh:
                    y_index = yi
                    break

            if x_index is not None and y_index is not None:
                data_list[y_index][x_index].append([x[i], y[i], data[i]])
                full_data_list[y_index][x_index].append([x[i], y[i], data[i], color_factor[i]])

        return data_list, full_data_list

    @staticmethod
    def create_stat_map(data_list):
        y_num = len(data_list)
        x_num = len(data_list[0])

        data_count = np.zeros((y_num, x_num))
        data_average = np.zeros((y_num, x_num))
        data_stddev = np.zeros((y_num, x_num))

        for y_idx in range(y_num):
            for x_idx in range(x_num):
                data_in_cell = data_list[y_idx][x_idx]
                if len(data_in_cell) == 0:
                    continue

                # Extract acceleration values (3rd column)
                acc_values = [d[2] for d in data_in_cell]

                data_count[y_idx, x_idx] = len(acc_values)
                data_average[y_idx, x_idx] = np.mean(acc_values)
                data_stddev[y_idx, x_idx] = np.std(acc_values)

        return data_count, data_average, data_stddev

    @staticmethod
    def extract_x_index_map(full_data_list, x_index):
        y_num = len(full_data_list)
        result_list = []

        for y_idx in range(y_num):
            if x_index < len(full_data_list[y_idx]):
                for data in full_data_list[y_idx][x_index]:
                    result_list.append(data)

        if len(result_list) == 0:
            return None

        return np.array(result_list)
