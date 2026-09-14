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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('robot_safety_monitor'))
    parameters = str(package_share / 'config' / 'safety.yaml')

    return LaunchDescription(
        [
            Node(
                package='robot_safety_monitor',
                executable='safety_monitor_node',
                name='safety_monitor',
                parameters=[parameters],
                output='screen',
            ),
            Node(
                package='robot_safety_monitor',
                executable='velocity_guard',
                name='velocity_guard',
                output='screen',
            ),
        ]
    )
