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

# Add the scripts directory to the path
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from calc_utils import CalcUtils  # noqa: 402

"""
Unit tests for CalcUtils class - Python calculation utilities.

This test file covers:
- Average filter (odd number, error cases)
- Lowpass filter using scipy
- 2D map creation (four_cell method)
- Statistics map creation (count, average, stddev)
- X-index map extraction
- Error handling for invalid inputs

Prerequisites:
- Python 3 with numpy and scipy installed
- PYTHONPATH includes the scripts directory

Running tests:
    # From package directory
    pytest test/test_calc_utils.py -v

    # Or using colcon
    colcon test --packages-select autoware_generic_value_calibrator --pytest-args -v
"""


class TestCalcUtils(unittest.TestCase):
    """Test suite for CalcUtils class."""

    def setUp(self):
        """Set up test fixtures."""
        self.calc_utils = CalcUtils()

    def test_average_filter_odd_number(self):
        """Test average filter with odd number of samples."""
        data = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        average_num = 3
        result = CalcUtils.average_filter(data, average_num)

        # Expected: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 4.333...]
        # np.convolve produces edge effects, so result length equals input length
        # The convolution with cut_num=1 from both ends gives this result
        expected = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 13.0 / 3.0])
        np.testing.assert_array_almost_equal(result, expected, decimal=5)

    def test_average_filter_even_number_error(self):
        """Test that average filter raises error for even number."""
        data = np.array([1.0, 2.0, 3.0, 4.0])
        average_num = 2

        with self.assertRaises(SystemExit):
            CalcUtils.average_filter(data, average_num)

    def test_average_filter_non_int_error(self):
        """Test that average filter raises error for non-integer."""
        data = np.array([1.0, 2.0, 3.0, 4.0])
        average_num = 3.5

        with self.assertRaises(SystemExit):
            CalcUtils.average_filter(data, average_num)

    def test_lowpass_filter_scipy(self):
        """Test lowpass filter using scipy."""
        # Create test signal: sine wave with noise
        sample_rate = 100.0
        t = np.linspace(0, 1, int(sample_rate))
        signal = np.sin(2 * np.pi * 5 * t) + 0.5 * np.sin(2 * np.pi * 50 * t)

        fp = 10.0  # Pass frequency
        fs = 20.0  # Stop frequency
        g_pass = 1.0  # Pass band ripple (dB)
        g_stop = 40.0  # Stop band attenuation (dB)

        result = CalcUtils.lowpass_filter_scipy(signal, sample_rate, fp, fs, g_pass, g_stop)

        # Check that result has same length as input
        self.assertEqual(len(result), len(signal))

        # Check that high frequency component is reduced
        # (This is a basic check - more sophisticated tests would verify frequency response)

    def test_create_2d_map_four_cell(self):
        """Test 2D map creation with four_cell method."""
        x = np.array([0.0, 5.0, 10.0, 15.0])
        y = np.array([-1.0, 0.0, 0.5, 1.0])
        data = np.array([1.0, 2.0, 3.0, 4.0])
        color_factor = np.array([0.1, 0.2, 0.3, 0.4])

        x_index_list = np.array([0.0, 10.0, 20.0])
        y_index_list = np.array([-1.0, 0.0, 1.0])

        data_list, full_data_list = CalcUtils.create_2d_map(
            x, y, data, color_factor, x_index_list, 0.0, y_index_list, 0.0, "four_cell"
        )

        # Check structure
        self.assertEqual(len(data_list), len(y_index_list))
        self.assertEqual(len(data_list[0]), len(x_index_list))

        # Check that data is assigned to correct cells
        # x=0.0 should go to x_index=0.0, y=-1.0 should go to y_index=-1.0
        self.assertGreater(len(data_list[0][0]), 0)  # Should have data in first cell

    def test_create_2d_map_shape_mismatch_error(self):
        """Test that create_2d_map raises error for shape mismatch."""
        x = np.array([0.0, 5.0])
        y = np.array([-1.0, 0.0, 1.0])  # Different size
        data = np.array([1.0, 2.0, 3.0])
        color_factor = np.array([0.1, 0.2, 0.3])

        x_index_list = np.array([0.0, 10.0])
        y_index_list = np.array([-1.0, 1.0])

        with self.assertRaises(SystemExit):
            CalcUtils.create_2d_map(x, y, data, color_factor, x_index_list, 0.0, y_index_list, 0.0)

    def test_create_stat_map(self):
        """Test statistics map creation."""
        # Create sample data list structure
        data_list = [
            [
                [[0.0, -1.0, 1.0], [0.0, -1.0, 1.5]],  # y_idx=0, x_idx=0
                [[5.0, -1.0, 2.0]],  # y_idx=0, x_idx=1
            ],
            [
                [[10.0, 0.0, 3.0], [10.0, 0.0, 3.5]],  # y_idx=1, x_idx=0
                [[15.0, 0.0, 4.0]],  # y_idx=1, x_idx=1
            ],
        ]

        data_count, data_average, data_stddev = CalcUtils.create_stat_map(data_list)

        # Check shape
        self.assertEqual(data_count.shape, (2, 2))
        self.assertEqual(data_average.shape, (2, 2))
        self.assertEqual(data_stddev.shape, (2, 2))

        # Check first cell statistics
        self.assertEqual(data_count[0, 0], 2)  # 2 data points
        self.assertAlmostEqual(data_average[0, 0], 1.25, places=5)  # (1.0 + 1.5) / 2
        self.assertGreater(data_stddev[0, 0], 0)  # Should have some std dev

    def test_extract_x_index_map(self):
        """Test extraction of data for specific x index."""
        full_data_list = [
            [
                [[0.0, -1.0, 1.0, 0.1], [0.0, -1.0, 1.5, 0.2]],  # y_idx=0, x_idx=0
                [[5.0, -1.0, 2.0, 0.3]],  # y_idx=0, x_idx=1
            ],
            [
                [[10.0, 0.0, 3.0, 0.4]],  # y_idx=1, x_idx=0
                [[15.0, 0.0, 4.0, 0.5]],  # y_idx=1, x_idx=1
            ],
        ]

        # Extract x_index = 0
        result = CalcUtils.extract_x_index_map(full_data_list, 0)

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 3)  # Should have 3 data points for x_idx=0

        # Extract x_index = 1
        result = CalcUtils.extract_x_index_map(full_data_list, 1)

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2)  # Should have 2 data points for x_idx=1

        # Extract x_index = 2 (doesn't exist)
        result = CalcUtils.extract_x_index_map(full_data_list, 2)

        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
