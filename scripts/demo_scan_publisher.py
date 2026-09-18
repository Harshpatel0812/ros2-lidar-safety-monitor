#!/usr/bin/env python3

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

"""Publish a synthetic LaserScan so the safety system can be tested without Gazebo."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class DemoScanPublisher(Node):
    """Publish a 180-degree scan with one configurable obstacle straight ahead."""

    def __init__(self) -> None:
        super().__init__('demo_scan_publisher')
        self.declare_parameter('obstacle_distance', 2.0)
        self._publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data
        )
        self._timer = self.create_timer(0.1, self._publish_scan)

    def _publish_scan(self) -> None:
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'demo_laser'
        message.angle_min = -math.pi / 2.0
        message.angle_max = math.pi / 2.0
        message.angle_increment = math.pi / 180.0
        message.scan_time = 0.1
        message.range_min = 0.10
        message.range_max = 10.0
        message.ranges = [5.0] * 181
        message.ranges[90] = float(
            self.get_parameter('obstacle_distance').value
        )
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoScanPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
