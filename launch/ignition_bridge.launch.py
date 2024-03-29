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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = 'diff_bot'
    world_name = 'diff_bot'

    base_link_frame = 'base_link'
    imu_frame = 'imu'
    lidar_frame = 'lidar'
    front_rgbd_frame = 'front_camera'
    back_rgb_frame = 'back_camera'
    third_rgb_frame = 'third_person_camera'

    ign_model_prefix = '/model/' + robot_name
    ign_world_prefix = '/world/' + world_name + ign_model_prefix
    ign_sensor_prefix = ign_world_prefix + '/link/base_link/sensor'

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=[robot_name],
            description='Namespace of ignition gazebo topic bridge'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['false'],
            description='Enable time resource from /clock'
        ),
        GroupAction([
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                namespace=namespace,
                output='screen',
                arguments=[
                    '/clock'
                    + '@rosgraph_msgs/msg/Clock'
                    + '[ignition.msgs.Clock'
                ],
                condition=IfCondition(use_sim_time)
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='control_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_model_prefix + '/cmd_vel'
                    + '@geometry_msgs/msg/Twist'
                    + ']ignition.msgs.Twist'
                ],
                remappings=[
                    (ign_model_prefix + '/cmd_vel', '/cmd_vel_nav')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='odometry_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_model_prefix + '/odometry'
                    + '@nav_msgs/msg/Odometry'
                    + '[ignition.msgs.Odometry'
                ],
                remappings=[
                    (ign_model_prefix + '/odometry', 'odom/raw')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='joint_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_world_prefix + '/joint_state'
                    + '@sensor_msgs/msg/JointState'
                    + '[ignition.msgs.Model'
                ],
                remappings=[
                    (ign_world_prefix + '/joint_state', 'joint_states')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='imu_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_sensor_prefix + '/imu/imu'
                    + '@sensor_msgs/msg/Imu'
                    + '[ignition.msgs.IMU'
                ],
                remappings=[
                    (ign_sensor_prefix + '/imu/imu', 'imu/data')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='lidar_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_sensor_prefix + '/lidar/scan/points'
                    + '@sensor_msgs/msg/PointCloud2'
                    + '[ignition.msgs.PointCloudPacked'
                ],
                remappings=[
                    (ign_sensor_prefix + '/lidar/scan/points', 'points')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='lidar_scan_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_sensor_prefix + '/lidar/scan'
                    + '@sensor_msgs/msg/LaserScan'
                    + '[ignition.msgs.LaserScan'
                ],
                remappings=[
                    (ign_sensor_prefix + '/lidar/scan', 'scan')
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='depth_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_sensor_prefix + '/front_camera/points'
                    + '@sensor_msgs/msg/PointCloud2'
                    + '[ignition.msgs.PointCloudPacked'
                ],
                remappings=[
                    (ign_sensor_prefix + '/front_camera/points', 'front_points')
                ]
            ),
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    ign_sensor_prefix + '/front_camera/image',
                    ign_sensor_prefix + '/back_camera/image',
                    ign_sensor_prefix + '/third_person_camera/image'
                ],
                remappings=[
                    (ign_sensor_prefix + '/front_camera/image', 'image/front'),
                    (ign_sensor_prefix + '/back_camera/image', 'image/back'),
                    (ign_sensor_prefix + '/third_person_camera/image', 'image/third_person')
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='imu_stf',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    imu_frame,
                    '--child-frame-id',
                    robot_name + '/'
                    + base_link_frame + '/'
                    + imu_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_stf',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    lidar_frame,
                    '--child-frame-id',
                    robot_name + '/'
                    + base_link_frame + '/'
                    + lidar_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='front_camera_stf',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    front_rgbd_frame,
                    '--child-frame-id',
                    robot_name + '/'
                    + base_link_frame + '/'
                    + front_rgbd_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='back_camera_stf',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    back_rgb_frame,
                    '--child-frame-id',
                    robot_name + '/'
                    + base_link_frame + '/'
                    + back_rgb_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='third_camera_stf',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    third_rgb_frame,
                    '--child-frame-id',
                    robot_name + '/'
                    + base_link_frame + '/'
                    + third_rgb_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_base',
                namespace=namespace,
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--frame-id',
                    'diff_bot/base_link',
                    '--child-frame-id',
                    'base_link',
                ]
            )
        ])
    ])
