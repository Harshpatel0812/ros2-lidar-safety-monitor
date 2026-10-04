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

"""Forward fresh velocity commands while enforcing the robot safety state."""

import copy
import math

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class VelocityGuard(Node):
    """Place a fail-safe command gate between a controller and mobile base."""

    def __init__(self) -> None:
        super().__init__('velocity_guard')

        self._stopped = True
        self._last_command_time = None
        self._command_fresh = False
        self._last_safety_time = None
        self._safety_fresh = False
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self._cmd_vel_timeout = float(
            self.declare_parameter('cmd_vel_timeout', 0.50).value
        )

        self._output_stamped = bool(
            self.declare_parameter('output_stamped', False).value
        )
        self._output_frame_id = str(
            self.declare_parameter(
                'output_frame_id',
                'base_link',
            ).value
        )

        self._safety_timeout = float(
            self.declare_parameter('safety_state_timeout', 0.50).value
        )

        for name, value in (
            ('cmd_vel_timeout', self._cmd_vel_timeout),
            ('safety_state_timeout', self._safety_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')

        if self._output_stamped and not self._output_frame_id:
            raise ValueError(
                'output_frame_id must not be empty '
                'when output_stamped is true'
            )

        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Wait for a live heartbeat rather than a retained clear state.
        heartbeat_qos = QoSProfile(depth=1)
        heartbeat_qos.reliability = ReliabilityPolicy.RELIABLE
        heartbeat_qos.durability = DurabilityPolicy.VOLATILE

        self._stop_subscription = self.create_subscription(
            Bool,
            '/safety/stop',
            self._stop_callback,
            heartbeat_qos,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self._command_callback,
            10,
        )

        output_message_type = (
            TwistStamped if self._output_stamped else Twist
        )

        self._command_publisher = self.create_publisher(
            output_message_type,
            '/cmd_vel',
            10,
        )
        self._freshness_publisher = self.create_publisher(
            Bool,
            '/safety/cmd_vel_fresh',
            safety_qos,
        )

        watchdog_period = max(
            0.01,
            min(
                0.10,
                self._cmd_vel_timeout / 2.0,
                self._safety_timeout / 2.0,
            ),
        )
        self._watchdog_timer = self.create_timer(
            watchdog_period,
            self._watchdog_callback,
            clock=self._watchdog_clock,
        )

        self._set_command_freshness(False, force=True)

        output_type = (
            'TwistStamped' if self._output_stamped else 'Twist'
        )

        self.get_logger().info(
            'Velocity guard ready; waiting for safety state and '
            f'velocity commands (timeout={self._cmd_vel_timeout:.2f} s, '
            f'output={output_type})'
        )

    def _stop_callback(self, message: Bool) -> None:
        """Update the safety state and stop immediately when blocked."""
        self._last_safety_time = self._watchdog_clock.now()
        self._safety_fresh = True
        changed = self._stopped != message.data
        self._stopped = message.data

        if self._stopped:
            self._publish_zero_velocity()

        if changed:
            state = 'BLOCKED' if self._stopped else 'ENABLED'
            self.get_logger().warn(
                f'Command output is now {state}'
            )

    def _command_callback(self, desired: Twist) -> None:
        """Record a fresh command and forward it only when safety allows."""
        self._last_command_time = self._watchdog_clock.now()
        self._set_command_freshness(True)

        if self._stopped or not self._safety_state_is_fresh():
            self._publish_zero_velocity()
            return

        self._publish_velocity(desired)

    def _watchdog_callback(self) -> None:
        """Publish zero velocity when the command stream becomes stale."""
        command_fresh = False

        if self._last_command_time is not None:
            command_age = (
                self._watchdog_clock.now() - self._last_command_time
            ).nanoseconds / 1e9

            command_fresh = (
                0.0 <= command_age <= self._cmd_vel_timeout
            )

        self._set_command_freshness(command_fresh)

        safety_fresh = self._safety_state_is_fresh()
        if self._safety_fresh and not safety_fresh:
            self.get_logger().warn(
                'Safety state timeout; blocking velocity commands'
            )
        self._safety_fresh = safety_fresh

        if self._stopped or not command_fresh or not safety_fresh:
            self._publish_zero_velocity()

    def _safety_state_is_fresh(self) -> bool:
        """Require a recent safety message using steady elapsed time."""
        if self._last_safety_time is None:
            return False

        age = (
            self._watchdog_clock.now() - self._last_safety_time
        ).nanoseconds / 1e9
        return 0.0 <= age <= self._safety_timeout

    def _publish_velocity(self, command: Twist) -> None:
        """Publish a Twist or stamped Twist using the configured format."""
        if self._output_stamped:
            output = TwistStamped()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = self._output_frame_id
            output.twist = copy.deepcopy(command)
            self._command_publisher.publish(output)
            return

        self._command_publisher.publish(copy.deepcopy(command))

    def _publish_zero_velocity(self) -> None:
        """Publish zero velocity using the configured output format."""
        self._publish_velocity(Twist())

    def _set_command_freshness(
        self,
        fresh: bool,
        force: bool = False,
    ) -> None:
        """Publish command freshness when its state changes."""
        if not force and fresh == self._command_fresh:
            return

        self._command_fresh = fresh

        status = Bool()
        status.data = fresh
        self._freshness_publisher.publish(status)

        if fresh:
            self.get_logger().info(
                'Velocity command stream is fresh'
            )
        elif self._last_command_time is not None:
            self.get_logger().warn(
                'Velocity command timeout; publishing zero velocity'
            )


def main(args=None) -> None:
    """Run the velocity guard node."""
    rclpy.init(args=args)
    node = VelocityGuard()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
