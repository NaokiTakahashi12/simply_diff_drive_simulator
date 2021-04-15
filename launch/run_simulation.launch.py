
import os
import sys
import glob

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
    LaunchService
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.frontend import Parser
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    Node,
    PushRosNamespace
)

def generate_launch_description():
    sim_pkg_share_dir = get_package_share_directory('simply_diff_drive_simulator')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('namespace',
            default_value = ['diff_bot'],
            description = 'Namespace of simply diff drive robot simulation (string)'
        ),
        DeclareLaunchArgument('output',
            default_value = ['screen'],
            description = 'Node output information'
        ),
        DeclareLaunchArgument('use_sim_time',
            default_value = ['true'],
            description = 'Enable time resource from /clock (true/false)'
        ),
        DeclareLaunchArgument('no_ign_gui',
            default_value = ['True'],
            description = 'Disable ignition gazebo gui (True/False)'
        ),
        DeclareLaunchArgument('with_navigation',
            default_value = ['true'],
            description = 'Launch with run_navigation.launch.py (true/false)'
        ),
        DeclareLaunchArgument('slam',
            default_value = ['true'],
            description = 'Enable sync slam by slam_toolbox (true/false)'
        ),
        DeclareLaunchArgument('with_rviz',
            default_value = ['true'],
            description = 'Launch with visualize.launch.xml (true/false)'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(
                    sim_pkg_share_dir,
                    'launch',
                    'ignition_gazebo.launch.py'
                )
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time
            }.items()
        ),
        Node(
            package = 'robot_localization',
            executable = 'ekf_node',
            name = 'ekf_node',
            namespace = namespace,
            output = 'screen',
            parameters = [
                {os.path.join(sim_pkg_share_dir, 'config', 'ekf.yaml')},
                {'use_sim_time': use_sim_time},
                {'odom_frame': 'diff_bot/odom'},
                {'base_link_frame': 'diff_bot/base_link'},
                {'odom0': 'odom/raw'},
                {'imu0': 'imu/data'}
            ],
            remappings = [
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
            launch_arguments = {
                'namespace': '',
                'use_sim_time': use_sim_time,
                'slam': LaunchConfiguration('slam'),
                'params_file': os.path.join(sim_pkg_share_dir, 'config', 'nav2.yaml'),
            }.items(),
            condition = IfCondition(LaunchConfiguration('with_navigation'))
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    sim_pkg_share_dir,
                    'launch',
                    'visualize.launch.xml'
                )
            ),
            launch_arguments = {
                'namespace': '',
                'use_sim_time': use_sim_time,
            }.items(),
            condition = IfCondition(LaunchConfiguration('with_rviz'))
        )
    ])

def main(argv = sys.argv[1:]):
    ld = generate_launch_description()

    ls = LaunchService(argv = argv)
    ls.include_launch_description(ld)

    return ls.run()

if __name__ == '__main__':
    sys.exit(main())

