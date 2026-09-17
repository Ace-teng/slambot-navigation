import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('landerpi_gazebo_sim')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    slam_pkg = get_package_share_directory('slam_toolbox')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    world = os.path.join(pkg, 'worlds', 'obstacle_world.world')
    urdf = os.path.join(pkg, 'urdf', 'landerpi_sim.urdf.xacro')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    map_file = os.path.join(pkg, 'maps', 'city_industrial.yaml')
    nav2_params = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')

    use_nav2 = LaunchConfiguration('nav2')
    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')
    use_demo = LaunchConfiguration('demo')
    use_slam = LaunchConfiguration('slam')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': use_gui, 'verbose': 'false'}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        condition=IfCondition(use_slam),
        launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_params}.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'localization_launch.py')),
        condition=UnlessCondition(use_slam),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav2', default_value='true', description='Start Nav2'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('demo', default_value='false', description='Run staged planning and obstacle demo'),
        DeclareLaunchArgument('slam', default_value='false', description='Build map online instead of using the prebuilt map'),
        gazebo,
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'landerpi', '-x', '-5.6', '-y', '-3.5', '-z', '0.08'],
        ),
        slam,
        localization,
        nav2,
        Node(
            package='landerpi_gazebo_sim', executable='initial_pose_publisher.py',
            name='landerpi_initial_pose', output='screen', condition=UnlessCondition(use_slam),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='landerpi_gazebo_sim', executable='staged_navigation_demo.py',
            name='staged_navigation_demo', output='screen', condition=IfCondition(use_demo),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(use_rviz), arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
