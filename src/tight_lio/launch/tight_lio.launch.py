from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('tight_lio')
    config  = os.path.join(pkg_dir, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'tight_lio.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config,
                              description='Full path to params yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulated time from /clock'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 with tight_lio config'),
        Node(
            package='tight_lio',
            executable='tight_lio_node',
            name='tight_lio_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
