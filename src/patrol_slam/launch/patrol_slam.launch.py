import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    patrol_slam_share = get_package_share_directory('patrol_slam')
    patrol_gazebo_share = get_package_share_directory('patrol_gazebo')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(
        patrol_slam_share,
        'config',
        'slam_params.yaml'
    )

    rviz_config = os.path.join(
        patrol_slam_share,
        'config',
        'patrol_slam.rviz'
    )

    patrol_sim_launch = os.path.join(
        patrol_gazebo_share,
        'launch',
        'patrol_sim.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Whether to launch RViz for SLAM visualization'
        ),

        # 1. Simulation stack: Gazebo + bridge + robot_state_publisher + spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(patrol_sim_launch)
        ),

        # 2. slam_toolbox: online asynchronous mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    slam_toolbox_share,
                    'launch',
                    'online_async_launch.py'
                )
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'true',
            }.items(),
        ),

        # 3. Optional RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])