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

# Copyright 2026 Harsh Patel
#
# Licensed under the Apache License, Version 2.0

"""Low-speed reactive driver for the warehouse safety demonstration."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class WarehouseDemoDriver(Node):
    """Drive slowly while steering toward the clearer side of the scan."""

    def __init__(self) -> None:
        super().__init__('warehouse_demo_driver')

        self._speed = float(
            self.declare_parameter('forward_speed', 0.10).value
        )
        self._turn_speed = float(
            self.declare_parameter('turn_speed', 0.35).value
        )
        self._scan_timeout = float(
            self.declare_parameter('scan_timeout', 0.50).value
        )
        self._clearance_limit = float(
            self.declare_parameter('clearance_limit', 0.85).value
        )

        if self._speed <= 0.0:
            raise ValueError('forward_speed must be positive')
        if self._turn_speed <= 0.0:
            raise ValueError('turn_speed must be positive')
        if self._scan_timeout <= 0.0:
            raise ValueError('scan_timeout must be positive')

        self._latest_scan = None
        self._last_scan_wall_time = None
        self._safety_stop = True
        self._turn_direction = 1.0

        self._scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10,
        )
        self._stop_subscription = self.create_subscription(
            Bool,
            '/safety/stop',
            self._stop_callback,
            10,
        )
        self._command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_raw',
            10,
        )

        self._timer = self.create_timer(0.10, self._control_callback)

        self.get_logger().info(
            'Warehouse demo driver ready; waiting for a valid scan '
            'and safety reset.'
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        self._latest_scan = scan
        self._last_scan_wall_time = time.monotonic()

    def _stop_callback(self, message: Bool) -> None:
        self._safety_stop = message.data
        if self._safety_stop:
            self._publish_zero()

    def _scan_is_fresh(self) -> bool:
        if self._latest_scan is None:
            return False
        if self._last_scan_wall_time is None:
            return False
        return (
            time.monotonic() - self._last_scan_wall_time
            <= self._scan_timeout
        )

    @staticmethod
    def _valid_range(scan: LaserScan, index: int) -> float:
        value = scan.ranges[index]

        if math.isnan(value):
            return scan.range_max

        if math.isinf(value):
            return scan.range_max if value > 0.0 else 0.0

        if value <= 0.0:
            return 0.0

        return min(value, scan.range_max)

    def _sector_minimum(
        self,
        scan: LaserScan,
        start_angle: float,
        end_angle: float,
    ) -> float:
        values = []

        for index in range(len(scan.ranges)):
            angle = scan.angle_min + index * scan.angle_increment
            angle = math.remainder(angle, 2.0 * math.pi)

            if start_angle <= angle <= end_angle:
                values.append(self._valid_range(scan, index))

        if not values:
            return 0.0

        return min(values)

    def _control_callback(self) -> None:
        if self._safety_stop or not self._scan_is_fresh():
            self._publish_zero()
            return

        scan = self._latest_scan
        front = self._sector_minimum(
            scan,
            math.radians(-25.0),
            math.radians(25.0),
        )
        left = self._sector_minimum(
            scan,
            math.radians(35.0),
            math.radians(100.0),
        )
        right = self._sector_minimum(
            scan,
            math.radians(-100.0),
            math.radians(-35.0),
        )

        if front < self._clearance_limit:
            if left >= right:
                self._turn_direction = 1.0
            else:
                self._turn_direction = -1.0

            command = Twist()
            command.angular.z = self._turn_direction * self._turn_speed
            self._command_publisher.publish(command)
            return

        command = Twist()
        command.linear.x = self._speed

        if left < self._clearance_limit:
            command.angular.z = -0.20
        elif right < self._clearance_limit:
            command.angular.z = 0.20

        self._command_publisher.publish(command)

    def _publish_zero(self) -> None:
        self._command_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WarehouseDemoDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
