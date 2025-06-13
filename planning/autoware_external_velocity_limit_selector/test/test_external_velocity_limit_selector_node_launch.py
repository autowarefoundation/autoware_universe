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

import os
import unittest

from ament_index_python import get_package_share_directory
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_internal_planning_msgs.msg import VelocityLimitClearCommand
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
import launch_testing
import pytest
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
import rclpy.qos

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    test_external_velocity_limit_selector_launch_file = os.path.join(
        get_package_share_directory("autoware_external_velocity_limit_selector"),
        "launch",
        "external_velocity_limit_selector.launch.xml",
    )
    external_velocity_limit_selector = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_external_velocity_limit_selector_launch_file),
    )

    return launch.LaunchDescription(
        [
            external_velocity_limit_selector,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestExternalVelocityLimitSelector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def velocity_limit_callback(self, msg):
        self.msg_buffer_ = msg

    def setUp(self):
        # Create a ROS node for tests
        self.test_node = rclpy.create_node("test_node")
        qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_api_limit_ = self.test_node.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity_default", qos
        )
        self.pub_internal_limit_ = self.test_node.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity_candidates", qos
        )
        self.pub_clear_limit_ = self.test_node.create_publisher(
            VelocityLimitClearCommand, "/planning/scenario_planning/clear_velocity_limit", qos
        )
        self.msg_buffer_ = None
        self.velocity_limit_output_ = None
        self.test_node.create_subscription(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity",
            self.velocity_limit_callback,
            1,
        )

    def wait_for_output(self):
        while not self.msg_buffer_:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        self.velocity_limit_output_ = self.msg_buffer_
        self.msg_buffer_ = None

    def tearDown(self):
        self.test_node.destroy_node()

    def update_max_vel_param(self, max_vel):
        set_params_client = self.test_node.create_client(
            SetParameters, "/external_velocity_limit_selector/set_parameters"
        )
        while not set_params_client.wait_for_service(timeout_sec=1.0):
            continue
        set_params_request = SetParameters.Request()
        set_params_request.parameters = [
            Parameter(
                name="max_vel",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_vel),
            ),
        ]
        future = set_params_client.call_async(set_params_request)
        rclpy.spin_until_future_complete(self.test_node, future)

        if future.result() is None:
            self.test_node.get_logger().error(
                "Exception while calling service: %r" % future.exception()
            )
            raise self.failureException("setting of initial parameters failed")

    @staticmethod
    def make_velocity_limit_msg(vel):
        msg = VelocityLimit()
        msg.use_constraints = False
        msg.max_velocity = vel
        return msg

    def test_external_velocity_limit_selector_node(self):
        self.update_max_vel_param(15.0)
        # clear velocity limit to trigger first output
        clear_cmd = VelocityLimitClearCommand(command=True)
        self.pub_clear_limit_.publish(clear_cmd)
        self.wait_for_output()
        # velocity limit is 0 before any limit is set
        self.assertEqual(self.velocity_limit_output_.max_velocity, 0.0)

        # Send velocity limits
        # new API velocity limit: higher than the node param -> limit is set to the param value
        api_limit = self.make_velocity_limit_msg(20.0)
        self.pub_api_limit_.publish(api_limit)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 15.0)
        # new API velocity limit
        api_limit = self.make_velocity_limit_msg(10.0)
        self.pub_api_limit_.publish(api_limit)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 10.0)
        # new INTERNAL velocity limit
        internal_limit = self.make_velocity_limit_msg(5.0)
        self.pub_internal_limit_.publish(internal_limit)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 5.0)
        # CLEAR: back to API velocity limit
        self.pub_clear_limit_.publish(clear_cmd)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 10.0)
        # lower the max_vel node parameter
        self.update_max_vel_param(2.5)
        self.pub_clear_limit_.publish(clear_cmd)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 2.5)
        # velocity limit set by internal limit is no longer applied since above max_vel parameter
        internal_limit = self.make_velocity_limit_msg(5.0)
        self.pub_internal_limit_.publish(internal_limit)
        self.wait_for_output()
        self.assertEqual(self.velocity_limit_output_.max_velocity, 2.5)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
