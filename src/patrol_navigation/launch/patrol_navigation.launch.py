import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    patrol_navigation_share = get_package_share_directory('patrol_navigation')
    patrol_gazebo_share = get_package_share_directory('patrol_gazebo')
    bt_navigator_share = get_package_share_directory('nav2_bt_navigator')

    patrol_sim_launch = os.path.join(
        patrol_gazebo_share,
        'launch',
        'patrol_sim.launch.py'
    )

    nav2_params = os.path.join(
        patrol_navigation_share,
        'config',
        'nav2_params.yaml'
    )

    rviz_config = os.path.join(
        patrol_navigation_share,
        'config',
        'patrol_navigation.rviz'
    )

    bt_xml = os.path.join(
        bt_navigator_share,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    map_yaml = LaunchConfiguration('map').perform(context)

    # Our patrol robot is spawned at the map origin by patrol_sim.launch.py.
    initial_x = 0.0
    initial_y = 0.0
    initial_yaw = 0.0

    return [
        # 1. Simulation stack: Gazebo + bridge + RSP + spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(patrol_sim_launch)
        ),

        # 2. Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                nav2_params,
                {
                    'yaml_filename': map_yaml,
                    'use_sim_time': True
                }
            ],
            output='screen',
        ),

        # 3. AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': True,
                    'initial_pose.x': initial_x,
                    'initial_pose.y': initial_y,
                    'initial_pose.yaw': initial_yaw
                }
            ],
            output='screen',
        ),

        # 4. Lifecycle manager for localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[
                {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'map_server',
                        'amcl'
                    ]
                }
            ],
            output='screen',
        ),

        # 5. Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': True
                }
            ],
            output='screen',
        ),

        # 6. Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': True
                }
            ],
            output='screen',
        ),

        # 7. Behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': True
                }
            ],
            output='screen',
        ),

        # 8. BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': True,
                    'default_nav_to_pose_bt_xml': bt_xml
                }
            ],
            output='screen',
        ),

        # 9. Lifecycle manager for navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'bt_navigator'
                    ]
                }
            ],
            output='screen',
        ),

        # 10. Optional RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                rviz_config
            ],
            parameters=[
                {
                    'use_sim_time': True
                }
            ],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/patrol_map.yaml'),
            description='Absolute path to the saved patrol map yaml file'
        ),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Whether to launch RViz for Nav2 visualization'
        ),

        OpaqueFunction(function=launch_setup),
    ])