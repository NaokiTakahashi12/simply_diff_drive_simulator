#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2023 NaokiTakahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable
)
from launch_ros.actions import Node


def generate_launch_description():
    namespace = 'diff_bot'
    sim_pkg_share_dir = get_package_share_directory('simply_diff_drive_simulator')

    model_name = 'simply_diff_drive_robot'
    models_search_prefix = sim_pkg_share_dir + '/models/**/'
    urdf_path = glob.glob(models_search_prefix + model_name + '*')[0]

    robot_name = 'diff_bot'

    return LaunchDescription([
        SetEnvironmentVariable(
            name='SDF_PATH',
            value=[
                EnvironmentVariable('SDF_PATH', default_value=''),
                os.path.join(sim_pkg_share_dir, 'models')
            ]
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
                os.path.join(sim_pkg_share_dir, 'worlds')
            ]
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value=['diff_bot'],
            description='Namespace of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['true'],
            description='Enable time resource from /clock (true/false)'
        ),
        DeclareLaunchArgument(
            'no_ign_gui',
            default_value=['false'],
            description='Disable ignition gazebo gui (true/false)'
        ),
        DeclareLaunchArgument(
            'ign_world_args',
            default_value=['-r diff_bot.sdf'],
            description='Ignition gazebo argments (string)',
            condition=UnlessCondition(LaunchConfiguration('no_ign_gui'))
        ),
        DeclareLaunchArgument(
            'ign_world_args',
            default_value=['-s -r diff_bot.sdf'],
            description='Ignition gazebo argments (string)',
            condition=IfCondition(LaunchConfiguration('no_ign_gui'))
        ),
        DeclareLaunchArgument(
            'with_bridge',
            default_value=['true'],
            description='Launch with ros_gz_briges (true/false)'
        ),
        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'ignore_timestamp': False},
                    {'use_tf_static': True},
                    {'publish_frequency': 20.0},
                    {'robot_description': open(urdf_path).read()}
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py'
                    )
                ]),
                launch_arguments={
                    'ign_args': LaunchConfiguration('ign_world_args')
                }.items()
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='robot_spawner',
                namespace=namespace,
                output='screen',
                arguments=[
                    '-name', robot_name,
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '1.0'
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    os.path.join(sim_pkg_share_dir, 'launch', 'ignition_bridge.launch.py')
                ]),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
                condition=IfCondition(LaunchConfiguration('with_bridge'))
            )
        ])
    ])
