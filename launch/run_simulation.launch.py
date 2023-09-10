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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
    LaunchService
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_share_dir = get_package_share_directory('simply_diff_drive_simulator')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=['diff_bot'],
            description='Namespace of simply diff drive robot simulation (string)'
        ),
        DeclareLaunchArgument(
            'output',
            default_value=['screen'],
            description='Node output information'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['true'],
            description='Enable time resource from /clock (true/false)'
        ),
        DeclareLaunchArgument(
            'no_ign_gui',
            default_value=['True'],
            description='Disable ignition gazebo gui (True/False)'
        ),
        DeclareLaunchArgument(
            'with_navigation',
            default_value=['true'],
            description='Launch with run_navigation.launch.py (true/false)'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value=['true'],
            description='Enable sync slam by slam_toolbox (true/false)'
        ),
        DeclareLaunchArgument(
            'with_rviz',
            default_value=['true'],
            description='Launch with visualize.launch.xml (true/false)'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(
                    sim_pkg_share_dir,
                    'launch',
                    'ignition_gazebo.launch.py'
                )
            ]),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time
            }.items()
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {os.path.join(sim_pkg_share_dir, 'config', 'ekf.yaml')},
                {'use_sim_time': use_sim_time},
                {'odom_frame': 'diff_bot/odom'},
                {'base_link_frame': 'diff_bot/base_link'},
                {'odom0': 'odom/raw'},
                {'imu0': 'imu/data'}
            ],
            remappings=[
               ('odometry/filtered', 'odom/filtered')
            ]
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    sim_pkg_share_dir,
                    'launch',
                    'run_navigation.launch.py'
                )
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
                'slam': LaunchConfiguration('slam'),
                'params_file': os.path.join(sim_pkg_share_dir, 'config', 'nav2.yaml'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('with_navigation'))
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    sim_pkg_share_dir,
                    'launch',
                    'visualize.launch.xml'
                )
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(LaunchConfiguration('with_rviz'))
        )
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)

    return ls.run()


if __name__ == '__main__':
    sys.exit(main())
