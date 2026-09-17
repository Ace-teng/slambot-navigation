"""Run JetRover simulation, online SLAM, and frontier exploration together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _prepare_output_directory(context):
    """Create the directory used by slam_toolbox's save_map service."""
    output_name = LaunchConfiguration('map_output').perform(context)
    expanded_name = os.path.abspath(os.path.expanduser(output_name))
    output_dir = os.path.dirname(expanded_name)
    os.makedirs(output_dir, exist_ok=True)
    return []


def generate_launch_description():
    simulation_share = get_package_share_directory('jetrover_description')
    navigation_share = get_package_share_directory('navigation')

    simulation_launch = os.path.join(
        simulation_share, 'launch', 'sim_gazebo.launch.py'
    )
    exploration_launch = os.path.join(
        navigation_share, 'launch', 'autonomous_exploration.launch.py'
    )
    default_world = os.path.join(
        simulation_share, 'worlds', 'turtlebot3_house.world'
    )
    default_output = os.path.join(
        os.path.expanduser('~'), '.ros', 'maps',
        'jetrover_exploration_map'
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    map_output = LaunchConfiguration('map_output')
    auto_save_map = LaunchConfiguration('auto_save_map')
    exploration_strategy = LaunchConfiguration('exploration_strategy')
    min_goal_distance = LaunchConfiguration('min_goal_distance')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    min_linear_speed = LaunchConfiguration('min_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    initial_scan_speed = LaunchConfiguration('initial_scan_speed')
    recovery_linear_speed = LaunchConfiguration('recovery_linear_speed')
    recovery_angular_speed = LaunchConfiguration('recovery_angular_speed')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'gui': gui,
            'rviz': rviz,
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(exploration_launch),
        launch_arguments={
            'use_sim_time': 'true',
            # gazebo.launch.py also declares a params_file argument. Passing
            # this explicitly prevents that global launch configuration from
            # shadowing the exploration package's default.
            'params_file': os.path.join(
                navigation_share, 'config', 'autonomous_exploration.yaml'
            ),
            'cmd_topic': 'cmd_vel',
            'auto_save_map': auto_save_map,
            'map_save_name': map_output,
            'exploration_strategy': exploration_strategy,
            'min_goal_distance': min_goal_distance,
            'max_linear_speed': max_linear_speed,
            'min_linear_speed': min_linear_speed,
            'max_angular_speed': max_angular_speed,
            'initial_scan_speed': initial_scan_speed,
            'recovery_linear_speed': recovery_linear_speed,
            'recovery_angular_speed': recovery_angular_speed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo Classic client',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Show SLAM and exploration in RViz2',
        ),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo world used as the unknown environment',
        ),
        DeclareLaunchArgument(
            'x_pose', default_value='3.50',
            description='Indoor JetRover start x coordinate in the house',
        ),
        DeclareLaunchArgument(
            'y_pose', default_value='2.00',
            description='Indoor JetRover start y coordinate in the house',
        ),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'exploration_strategy', default_value='dfs',
            description='Use directional depth-first frontier ordering',
        ),
        DeclareLaunchArgument(
            'min_goal_distance', default_value='1.20',
            description='Ignore nearby frontiers to avoid short oscillations',
        ),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='0.35',
            description='Maximum simulation forward speed in m/s',
        ),
        DeclareLaunchArgument(
            'min_linear_speed', default_value='0.07',
            description='Minimum simulation forward speed in m/s',
        ),
        DeclareLaunchArgument(
            'max_angular_speed', default_value='0.90',
            description='Maximum simulation turn speed in rad/s',
        ),
        DeclareLaunchArgument(
            'initial_scan_speed', default_value='0.45',
            description='Initial mapping scan speed in rad/s',
        ),
        DeclareLaunchArgument(
            'recovery_linear_speed', default_value='0.08',
            description='Simulation recovery reverse speed in m/s',
        ),
        DeclareLaunchArgument(
            'recovery_angular_speed', default_value='0.65',
            description='Simulation recovery turn speed in rad/s',
        ),
        DeclareLaunchArgument(
            'auto_save_map', default_value='true',
            description='Save the map after exploration completes',
        ),
        DeclareLaunchArgument(
            'map_output', default_value=default_output,
            description='Saved occupancy-map path without suffixes',
        ),
        OpaqueFunction(function=_prepare_output_directory),
        simulation,
        # The node also waits for map/scan/TF, while this delay avoids noisy
        # transient warnings during Gazebo entity insertion and SLAM startup.
        TimerAction(period=5.0, actions=[exploration]),
    ])
