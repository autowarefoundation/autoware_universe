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


def load_indices_from_csv(csv_path):
    """
    Load value and velocity indices from CSV file.

    Returns (value_list, vel_list) as numpy arrays.
    """
    try:
        with open(csv_path, "r") as f:
            lines = f.readlines()

        if len(lines) < 2:
            print(f"Warning: CSV file {csv_path} has insufficient data")
            return get_default_indices()

        # First row contains velocity indices (skip first column which is label)
        vel_header = lines[0].strip().split(",")
        vel_list = np.array([float(x) for x in vel_header[1:]])

        # First column contains value indices (skip first row which is header)
        value_list = []
        for line in lines[1:]:
            if len(line.strip()) == 0:
                continue
            parts = line.strip().split(",")
            if len(parts) > 0:
                try:
                    value_list.append(float(parts[0]))
                except ValueError:
                    pass
        value_list = np.array(value_list)

        print(f"Loaded indices from {csv_path}:")
        print(
            f"  Value range: [{value_list.min():.3f}, {value_list.max():.3f}] with {len(value_list)} points"
        )
        print(
            f"  Velocity range: [{vel_list.min():.3f}, {vel_list.max():.3f}] m/s with {len(vel_list)} points"
        )

        return value_list, vel_list

    except Exception as e:
        print(f"Warning: Could not load indices from {csv_path}: {e}")
        print("Using default indices")
        return get_default_indices()


def get_default_indices():
    """Get default value and velocity indices."""
    # Default value range: -1.0 to 1.0
    VALUE_LIST = np.linspace(-1.0, 1.0, 11)
    # Default velocity range: 0 to 20 m/s
    VEL_LIST = np.linspace(0, 20, 11)
    return VALUE_LIST, VEL_LIST


# Global variables (will be overwritten when loading from CSV)
VALUE_LIST, VEL_LIST = get_default_indices()
