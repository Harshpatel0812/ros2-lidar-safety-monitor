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

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    safety_monitor = launch_ros.actions.Node(
        package='robot_safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor',
        parameters=[
            {
                'stop_distance': 0.45,
                'release_distance': 0.60,
                'field_of_view_degrees': 60.0,
                'scan_timeout': 0.30,
            }
        ],
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                safety_monitor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'safety_monitor': safety_monitor},
    )


class TestWatchdogIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('watchdog_integration_test')

        cls.scan_publisher = cls.node.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data,
        )

        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.last_stop_state = None

        cls.stop_subscription = cls.node.create_subscription(
            Bool,
            '/safety/stop',
            cls.stop_callback,
            stop_qos,
        )

        cls.reset_client = cls.node.create_client(
            Trigger,
            '/safety/reset',
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def stop_callback(cls, message):
        cls.last_stop_state = message.data

    def wait_until(self, condition, timeout=3.0):
        end_time = time.monotonic() + timeout

        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if condition():
                return True

        return False

    def publish_clear_scan(self):
        message = LaserScan()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'test_laser'

        message.angle_min = -math.pi / 2.0
        message.angle_max = math.pi / 2.0
        message.angle_increment = math.pi / 180.0

        message.range_min = 0.10
        message.range_max = 10.0
        message.ranges = [2.0] * 181

        self.scan_publisher.publish(message)

    def call_reset_service(self):
        request = Trigger.Request()
        future = self.reset_client.call_async(request)

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=3.0,
        )

        self.assertTrue(future.done())
        self.assertIsNotNone(future.result())

        return future.result()

    def test_lidar_timeout_triggers_safety_stop(self):
        self.assertTrue(
            self.reset_client.wait_for_service(timeout_sec=5.0)
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.scan_publisher.get_subscription_count() > 0
            )
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.last_stop_state is True
            )
        )

        for _ in range(10):
            self.publish_clear_scan()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        reset_response = self.call_reset_service()

        self.assertTrue(reset_response.success)

        self.assertTrue(
            self.wait_until(
                lambda: self.last_stop_state is False
            )
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.last_stop_state is True,
                timeout=2.0,
            )
        )

        stale_reset_response = self.call_reset_service()

        self.assertFalse(stale_reset_response.success)
        self.assertIn(
            'missing or stale',
            stale_reset_response.message,
        )
