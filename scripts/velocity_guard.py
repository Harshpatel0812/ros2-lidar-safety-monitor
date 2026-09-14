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

"""Forward safe velocity commands and suppress motion during a safety stop."""

import copy

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class VelocityGuard(Node):
    """Place a safety gate between a planner/teleop node and the mobile base."""

    def __init__(self) -> None:
        super().__init__('velocity_guard')
        self._stopped = True  # Fail safe until the monitor publishes its state.

        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._stop_subscription = self.create_subscription(
            Bool, '/safety/stop', self._stop_callback, stop_qos
        )
        self._command_subscription = self.create_subscription(
            Twist, '/cmd_vel_raw', self._command_callback, 10
        )
        self._command_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.get_logger().info(
            'Velocity guard ready; waiting for safety state'
        )

    def _stop_callback(self, message: Bool) -> None:
        changed = self._stopped != message.data
        self._stopped = message.data

        if changed:
            state = 'BLOCKED' if self._stopped else 'ENABLED'
            self.get_logger().warn(
                f'Command output is now {state}'
            )

    def _command_callback(self, desired: Twist) -> None:
        output = Twist() if self._stopped else copy.deepcopy(desired)
        self._command_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGuard()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
