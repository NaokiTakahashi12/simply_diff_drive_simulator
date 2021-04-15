
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

    nav2_bringup_pkg = 'nav2_bringup'
    nav2_bringup_pkg_share_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument('namespace',
            default_value = [''],
            description = 'Namespace of simply diff drive robot simulation'
        ),
        DeclareLaunchArgument('use_sim_time',
            default_value = ['true'],
            description = 'Enable time resource from /clock (true/false)'
        ),
        DeclareLaunchArgument('autostart',
            default_value = ['true'],
            description = 'Enable autostart navigation2 lifecycle manager (true/false)'
        ),
        DeclareLaunchArgument('params_file',
            default_value = os.path.join(
                sim_pkg_share_dir, 'config', 'nav2.yaml'
            ),
            description = 'Navigation2 parameter file path (Full path)'
        ),
        DeclareLaunchArgument('default_bt_xml_filename',
            default_value = os.path.join(
                get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
            ),
            description = 'Navigation2 behavior tree parameter file path (Full path)'
        ),
        DeclareLaunchArgument('map',
            default_value = os.path.join(
                sim_pkg_share_dir, 'maps', 'simulator_map.yaml'
            ),
            description = 'Map config file path for 2D map_server (Full path)'
        ),
        DeclareLaunchArgument('slam',
            default_value = ['true'],
            description = 'Enable sync slam by slam_toolbox (true/false)'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg_share_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
                'map_subscribe_transient_local': 'true'
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg_share_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'map': LaunchConfiguration('map')
            }.items(),
            condition = UnlessCondition(slam)
        ),
        GroupAction([
            Node(
                package = 'slam_toolbox',
                executable = 'async_slam_toolbox_node',
                name = 'slam_toolbox',
                output = 'screen',
                parameters = [
                    os.path.join(
                        sim_pkg_share_dir, 'config', 'slam_toolbox.yaml'
                    ),
                    {'use_sim_time': use_sim_time}
                ],
                condition = IfCondition(slam)
            ),
            Node(
                package = 'nav2_map_server',
                executable = 'map_saver_server',
                name = 'map_saver',
                output = 'screen',
                parameters = [
                    params_file
                ],
                condition = IfCondition(slam)
            ),
            Node(
                package = 'nav2_lifecycle_manager',
                executable = 'lifecycle_manager',
                name = 'lifecycle_manager_slam',
                output = 'screen',
                parameters = [
                    params_file,
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_saver']}
                ],
                condition = IfCondition(slam)
            )
        ])
       #IncludeLaunchDescription(
       #    PythonLaunchDescriptionSource(
       #        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
       #    ),
       #    launch_arguments = {
       #        'namespace': '',
       #        'use_sim_time': use_sim_time,
       #        'map': LaunchConfiguration('map'),
       #        'slam': LaunchConfiguration('slam'),
       #        'params_file': LaunchConfiguration('params_file'),
       #    }.items()
       #),
    ])
