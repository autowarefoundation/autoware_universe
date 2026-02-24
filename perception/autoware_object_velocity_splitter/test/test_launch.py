# Copyright 2025 TIER IV, Inc.
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

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for standalone mode test."""
    launch_file = os.path.join(
        get_package_share_directory("autoware_object_velocity_splitter"),
        "launch",
        "object_velocity_splitter.launch.xml",
    )

    # Launch in standalone mode (container_name is empty by default)
    node_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file),
    )

    return launch.LaunchDescription(
        [
            node_launch,
            # Shutdown after 3 seconds
            TimerAction(period=3.0, actions=[launch.actions.Shutdown()]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLaunchStandalone(unittest.TestCase):
    """Test standalone mode launch (container_name is empty)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")

    def tearDown(self):
        self.test_node.destroy_node()

    def test_node_exists(self):
        """Test that the node is running."""
        # Wait for node to be discovered
        end_time = time.time() + 2.0
        node_found = False

        while time.time() < end_time and not node_found:
            node_names = self.test_node.get_node_names()
            if "object_velocity_splitter" in node_names:
                node_found = True
            else:
                rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertTrue(node_found, "object_velocity_splitter node not found")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Test process exit code after shutdown."""

    def test_exit_code(self, proc_info):
        """Check that all processes exit with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
