# Copyright 2026 Harsh Patel
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

import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    velocity_guard = launch_ros.actions.Node(
        package='robot_safety_monitor',
        executable='velocity_guard',
        name='velocity_guard',
        parameters=[
            {
                'cmd_vel_timeout': 0.50,
            }
        ],
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                velocity_guard,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'velocity_guard': velocity_guard},
    )


class TestVelocityGuardIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('velocity_guard_integration_test')

        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.stop_publisher = cls.node.create_publisher(
            Bool,
            '/safety/stop',
            safety_qos,
        )
        cls.command_publisher = cls.node.create_publisher(
            Twist,
            '/cmd_vel_raw',
            10,
        )

        cls.heartbeat_state = True
        cls.heartbeat_timer = cls.node.create_timer(
            0.05,
            cls.heartbeat_callback,
        )
        cls.last_freshness = None
        cls.last_output = None

        cls.freshness_subscription = cls.node.create_subscription(
            Bool,
            '/safety/cmd_vel_fresh',
            cls.freshness_callback,
            safety_qos,
        )
        cls.output_subscription = cls.node.create_subscription(
            Twist,
            '/cmd_vel',
            cls.output_callback,
            10,
        )

    @classmethod
    def heartbeat_callback(cls):
        message = Bool()
        message.data = cls.heartbeat_state
        cls.stop_publisher.publish(message)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def freshness_callback(cls, message):
        cls.last_freshness = message.data

    @classmethod
    def output_callback(cls, message):
        cls.last_output = message

    def wait_until(self, condition, timeout=3.0):
        end_time = time.monotonic() + timeout

        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if condition():
                return True

        return False

    def publish_stop_state(self, stopped):
        type(self).heartbeat_state = stopped
        message = Bool()
        message.data = stopped

        for _ in range(3):
            self.stop_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_command(self, linear_x, angular_z):
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.command_publisher.publish(message)

    def output_matches(self, linear_x, angular_z):
        if self.last_output is None:
            return False

        tolerance = 1e-6

        return (
            abs(self.last_output.linear.x - linear_x) < tolerance
            and abs(self.last_output.angular.z - angular_z) < tolerance
        )

    def output_is_zero(self):
        if self.last_output is None:
            return False

        values = [
            self.last_output.linear.x,
            self.last_output.linear.y,
            self.last_output.linear.z,
            self.last_output.angular.x,
            self.last_output.angular.y,
            self.last_output.angular.z,
        ]

        return all(abs(value) < 1e-6 for value in values)

    def test_velocity_command_watchdog(self):
        self.assertTrue(
            self.wait_until(
                lambda: self.stop_publisher.get_subscription_count() > 0
            )
        )
        self.assertTrue(
            self.wait_until(
                lambda: self.command_publisher.get_subscription_count() > 0
            )
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.last_freshness is False
            )
        )
        self.assertTrue(
            self.wait_until(self.output_is_zero)
        )

        self.publish_stop_state(False)
        self.publish_command(0.60, 0.20)

        self.assertTrue(
            self.wait_until(
                lambda: self.last_freshness is True
            )
        )
        self.assertTrue(
            self.wait_until(
                lambda: self.output_matches(0.60, 0.20)
            )
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.last_freshness is False,
                timeout=2.0,
            )
        )
        self.assertTrue(
            self.wait_until(self.output_is_zero)
        )

        self.publish_command(0.40, -0.10)

        self.assertTrue(
            self.wait_until(
                lambda: self.last_freshness is True
            )
        )
        self.assertTrue(
            self.wait_until(
                lambda: self.output_matches(0.40, -0.10)
            )
        )

        self.publish_stop_state(True)

        self.assertIs(self.last_freshness, True)
        self.assertTrue(
            self.wait_until(
                self.output_is_zero,
                timeout=0.40,
            )
        )

    def test_z_safety_heartbeat_timeout(self):
        """Block fresh commands when safety updates disappear."""
        self.assertTrue(
            self.wait_until(
                lambda: self.command_publisher.get_subscription_count() > 0
                and self.stop_publisher.get_subscription_count() > 0
            )
        )

        self.publish_stop_state(False)
        command_timer = self.node.create_timer(
            0.05,
            lambda: self.publish_command(0.12, 0.0),
        )

        try:
            self.assertTrue(
                self.wait_until(
                    lambda: self.output_matches(0.12, 0.0)
                )
            )

            # Simulate monitor loss while velocity commands keep arriving.
            self.heartbeat_timer.cancel()

            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)

            type(self).last_output = None
            self.assertTrue(
                self.wait_until(self.output_is_zero, timeout=1.0)
            )
            self.assertIs(self.last_freshness, True)

            # Check sustained blocking, not just one zero message.
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                self.assertTrue(self.output_is_zero())

            # A restarted monitor initially reports STOP.
            type(self).heartbeat_state = True
            self.heartbeat_timer.reset()
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                self.assertTrue(self.output_is_zero())

            # Simulate a subsequently accepted explicit reset.
            self.publish_stop_state(False)
            self.assertTrue(
                self.wait_until(
                    lambda: self.output_matches(0.12, 0.0)
                )
            )
        finally:
            self.node.destroy_timer(command_timer)
            self.publish_stop_state(True)


@launch_testing.post_shutdown_test()
class TestGuardShutdown(unittest.TestCase):
    """Require a clean exit after the launch test shuts down the guard."""

    def test_exit_code(self, proc_info, velocity_guard):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=velocity_guard,
        )
