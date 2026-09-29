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

"""Launch the complete TurtleBot3 warehouse safety demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the complete warehouse safety demonstration."""
    package_share = Path(
        get_package_share_directory('robot_safety_monitor')
    )
    turtlebot_share = Path(
        get_package_share_directory('turtlebot3_gazebo')
    )
    ros_gz_sim_share = Path(
        get_package_share_directory('ros_gz_sim')
    )

    world_path = (
        package_share /
        'worlds' /
        'warehouse_safety_demo.sdf'
    )
    model_resource_path = (
        package_share /
        'third_party' /
        'warehouse_simulation_toolkit' /
        'models'
    )
    safety_parameters = (
        package_share /
        'config' /
        'safety.yaml'
    )
    rviz_config = (
        package_share /
        'rviz' /
        'safety_monitor.rviz'
    )

    turtlebot_model = 'burger'
    turtlebot_sdf = (
        turtlebot_share /
        'models' /
        f'turtlebot3_{turtlebot_model}' /
        'model.sdf'
    )
    turtlebot_urdf = (
        turtlebot_share /
        'urdf' /
        f'turtlebot3_{turtlebot_model}.urdf'
    )
    bridge_parameters = (
        turtlebot_share /
        'params' /
        f'turtlebot3_{turtlebot_model}_bridge.yaml'
    )

    robot_description = turtlebot_urdf.read_text()

    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                ros_gz_sim_share /
                'launch' /
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                ros_gz_sim_share /
                'launch' /
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }
        ],
    )

    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-name',
            turtlebot_model,
            '-file',
            str(turtlebot_sdf),
            '-x',
            '-6.0',
            '-y',
            '0.0',
            '-z',
            '0.01',
            '-Y',
            '-1.3963',
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_parameters}',
        ],
    )

    safety_monitor = Node(
        package='robot_safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor',
        output='screen',
        parameters=[
            str(safety_parameters),
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    velocity_guard = Node(
        package='robot_safety_monitor',
        executable='velocity_guard',
        name='velocity_guard',
        output='screen',
        parameters=[
            str(safety_parameters),
            {
                'use_sim_time': use_sim_time,
                'output_stamped': True,
                'output_frame_id': 'base_link',
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='safety_monitor_rviz',
        output='screen',
        arguments=[
            '-d',
            str(rviz_config),
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                description='Start RViz with the saved safety display.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock.',
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                str(model_resource_path),
            ),
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_turtlebot,
            gazebo_bridge,
            safety_monitor,
            velocity_guard,
            rviz,
        ]
    )
