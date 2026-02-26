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

import numpy as np


class CSVReader:
    def __init__(self, csv_file, csv_type="file"):
        if csv_type == "file":
            with open(csv_file) as f:
                self.data_text = f.readlines()
        elif csv_type == "text":
            self.data_text = csv_file.split("\n")

        if len(self.data_text[0]) == 0:
            return

        self.raw_data_array = []
        for line in self.data_text:
            if len(line) <= 1:
                continue
            line_split = line.split(",")
            csv_data = []
            for i, word in enumerate(line_split):
                try:
                    csv_data.append(float(word))
                except ValueError:
                    pass
            if len(csv_data) > 0:
                self.raw_data_array.append(csv_data)

        self.raw_data_array = np.array(self.raw_data_array)

    def removeUnusedData(
        self,
        min_vel_thr,
        max_steer_thr,
        max_pitch_thr,
        max_value_vel_thr,
        max_jerk_thr,
    ):
        # Filter data based on thresholds
        # Column indices: timestamp(0), velocity(1), accel(2), pitch_comp_accel(3),
        #                 input_value(4), input_value_speed(5), pitch(6), steer(7), jerk(8)
        vel_data = self.raw_data_array[:, 1]
        steer_data = np.abs(self.raw_data_array[:, 7])
        pitch_data = np.abs(self.raw_data_array[:, 6])
        value_speed_data = np.abs(self.raw_data_array[:, 5])
        jerk_data = np.abs(self.raw_data_array[:, 8])

        valid_indices = np.where(
            (vel_data > min_vel_thr)
            & (steer_data < max_steer_thr)
            & (pitch_data < max_pitch_thr)
            & (value_speed_data < max_value_vel_thr)
            & (jerk_data < max_jerk_thr)
        )[0]

        self.data_array = self.raw_data_array[valid_indices]
        return self.data_array

    def getVelData(self):
        return self.data_array[:, 1]

    def getValueData(self):
        return self.data_array[:, 4]

    def getAccData(self):
        return self.data_array[:, 3]  # pitch-compensated acceleration

    def getPitchData(self):
        return self.data_array[:, 6]
