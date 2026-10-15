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

"""Run the approach, latched-stop, and recovery recording sequence."""

import subprocess
import time


def move_obstacle(x, y):
    """Position the cylinder through the Gazebo world service."""
    request = (
        'name: "crossing_obstacle", '
        f'position: {{x: {x}, y: {y}, z: 0.85}}, '
        'orientation: {w: 1.0}'
    )
    result = subprocess.run(
        [
            'gz', 'service',
            '-s', '/world/warehouse_safety_demo/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', request,
        ],
        capture_output=True,
        text=True,
        timeout=5,
        check=True,
    )
    if 'data: true' not in result.stdout:
        raise RuntimeError('Obstacle move rejected: ' + result.stdout)


def reset_safety():
    """Request a reset and abort if the monitor rejects it."""
    result = subprocess.run(
        [
            'ros2', 'service', 'call',
            '/safety/reset', 'std_srvs/srv/Trigger', '{}',
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )
    print(result.stdout, flush=True)
    if 'success=True' not in result.stdout:
        raise RuntimeError('Safety reset rejected; sequence aborted.')


def drive(seconds):
    """Publish a bounded forward-command stream through the guard."""
    result = subprocess.run(
        [
            'timeout', '--signal=INT', f'{seconds}s',
            'ros2', 'topic', 'pub', '--rate', '10',
            '/cmd_vel_raw', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.10}, angular: {z: 0.0}}',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=seconds + 10,
        check=False,
    )
    if result.returncode not in (0, 124):
        raise RuntimeError('Command publisher failed: ' + result.stderr)


def main():
    """Run the visual demonstration without measuring response latency."""
    print('Use a fresh simulation with the demo driver disabled.', flush=True)
    move_obstacle(-5.8437, -0.8863)
    time.sleep(2)
    reset_safety()

    print('Motion starts in 5 seconds. Show Gazebo and RViz.', flush=True)
    time.sleep(5)
    print('Approach: publishing forward commands for 8 seconds.', flush=True)
    drive(8)

    print('Holding for 3 seconds.', flush=True)
    time.sleep(3)
    move_obstacle(-5.0558, -0.7473)

    print('Obstacle moved aside; holding the latch for 5 seconds.', flush=True)
    time.sleep(5)
    reset_safety()
    time.sleep(2)

    print('Recovery: publishing forward commands for 3 seconds.', flush=True)
    drive(3)
    time.sleep(3)
    print('Finished. Verify the robot stopped, then end recording.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted; command forwarding expires via the watchdog.')
