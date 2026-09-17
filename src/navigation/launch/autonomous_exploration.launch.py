"""Launch autonomous exploration beside an already-running SLAM stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation_share = get_package_share_directory('navigation')
    default_params = os.path.join(
        navigation_share,
        'config',
        'autonomous_exploration.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    cmd_topic = LaunchConfiguration('cmd_topic')
    auto_save_map = LaunchConfiguration('auto_save_map')
    map_save_name = LaunchConfiguration('map_save_name')
    exploration_strategy = LaunchConfiguration('exploration_strategy')
    min_goal_distance = LaunchConfiguration('min_goal_distance')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    min_linear_speed = LaunchConfiguration('min_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    initial_scan_speed = LaunchConfiguration('initial_scan_speed')
    recovery_linear_speed = LaunchConfiguration('recovery_linear_speed')
    recovery_angular_speed = LaunchConfiguration('recovery_angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock when true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Autonomous exploration parameter file',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace; relative topics inherit it',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global map TF frame',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry TF frame',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base TF frame',
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='controller/cmd_vel',
            description='Velocity command topic',
        ),
        DeclareLaunchArgument(
            'auto_save_map',
            default_value='true',
            description='Save the completed occupancy map automatically',
        ),
        DeclareLaunchArgument(
            'map_save_name',
            default_value='exploration_map',
            description='Output path without .pgm/.yaml suffixes',
        ),
        DeclareLaunchArgument(
            'exploration_strategy',
            default_value='dfs',
            description='Frontier ordering: dfs or utility',
        ),
        DeclareLaunchArgument('min_goal_distance', default_value='0.80'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.20'),
        DeclareLaunchArgument('min_linear_speed', default_value='0.04'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.50'),
        DeclareLaunchArgument('initial_scan_speed', default_value='0.25'),
        DeclareLaunchArgument('recovery_linear_speed', default_value='0.06'),
        DeclareLaunchArgument('recovery_angular_speed', default_value='0.35'),
        Node(
            package='navigation',
            executable='autonomous_exploration',
            name='autonomous_exploration',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'cmd_topic': cmd_topic,
                    'auto_save_map': auto_save_map,
                    'map_save_name': map_save_name,
                    'exploration_strategy': exploration_strategy,
                    'min_goal_distance': ParameterValue(
                        min_goal_distance, value_type=float
                    ),
                    'max_linear_speed': ParameterValue(
                        max_linear_speed, value_type=float
                    ),
                    'min_linear_speed': ParameterValue(
                        min_linear_speed, value_type=float
                    ),
                    'max_angular_speed': ParameterValue(
                        max_angular_speed, value_type=float
                    ),
                    'initial_scan_speed': ParameterValue(
                        initial_scan_speed, value_type=float
                    ),
                    'recovery_linear_speed': ParameterValue(
                        recovery_linear_speed, value_type=float
                    ),
                    'recovery_angular_speed': ParameterValue(
                        recovery_angular_speed, value_type=float
                    ),
                },
            ],
        ),
    ])
