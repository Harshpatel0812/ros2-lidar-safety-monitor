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
from geometry_msgs.msg import TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


STOP_TOPIC = '/stamped_test/safety/stop'
RAW_COMMAND_TOPIC = '/stamped_test/cmd_vel_raw'
OUTPUT_TOPIC = '/stamped_test/cmd_vel'
FRESHNESS_TOPIC = '/stamped_test/safety/cmd_vel_fresh'


@pytest.mark.launch_test
def generate_test_description():
    velocity_guard = launch_ros.actions.Node(
        package='robot_safety_monitor',
        executable='velocity_guard',
        name='velocity_guard_stamped',
        parameters=[
            {
                'cmd_vel_timeout': 0.50,
                'output_stamped': True,
                'output_frame_id': 'base_link',
            }
        ],
        remappings=[
            ('/safety/stop', STOP_TOPIC),
            ('/cmd_vel_raw', RAW_COMMAND_TOPIC),
            ('/cmd_vel', OUTPUT_TOPIC),
            ('/safety/cmd_vel_fresh', FRESHNESS_TOPIC),
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


class TestVelocityGuardStampedIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'velocity_guard_stamped_integration_test'
        )

        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.stop_publisher = cls.node.create_publisher(
            Bool,
            STOP_TOPIC,
            safety_qos,
        )
        cls.command_publisher = cls.node.create_publisher(
            Twist,
            RAW_COMMAND_TOPIC,
            10,
        )

        cls.last_freshness = None
        cls.last_output = None

        cls.freshness_subscription = cls.node.create_subscription(
            Bool,
            FRESHNESS_TOPIC,
            cls.freshness_callback,
            safety_qos,
        )
        cls.output_subscription = cls.node.create_subscription(
            TwistStamped,
            OUTPUT_TOPIC,
            cls.output_callback,
            10,
        )

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
            abs(self.last_output.twist.linear.x - linear_x) < tolerance
            and abs(
                self.last_output.twist.angular.z - angular_z
            ) < tolerance
        )

    def output_is_zero(self):
        if self.last_output is None:
            return False

        twist = self.last_output.twist
        values = [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ]

        return all(abs(value) < 1e-6 for value in values)

    def stamp_is_valid(self):
        if self.last_output is None:
            return False

        stamp = self.last_output.header.stamp
        return stamp.sec > 0 or stamp.nanosec > 0

    def test_stamped_velocity_output(self):
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
        self.publish_command(0.35, -0.15)

        self.assertTrue(
            self.wait_until(
                lambda: self.last_freshness is True
            )
        )
        self.assertTrue(
            self.wait_until(
                lambda: self.output_matches(0.35, -0.15)
            )
        )

        self.assertEqual(
            self.last_output.header.frame_id,
            'base_link',
        )
        self.assertTrue(self.stamp_is_valid())

        self.publish_stop_state(True)

        self.assertTrue(
            self.wait_until(
                self.output_is_zero,
                timeout=0.40,
            )
        )
        self.assertEqual(
            self.last_output.header.frame_id,
            'base_link',
        )
        self.assertTrue(self.stamp_is_valid())
