import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# slam_base.launch.py 读取该环境变量决定用 install share 还是源码路径
os.environ.setdefault('need_compile', 'True')


def generate_launch_description():
    pkg_share = get_package_share_directory('jetrover_description')

    gui = LaunchConfiguration('gui')
    rviz_enabled = LaunchConfiguration('rviz')
    world_file = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    xacro_file = os.path.join(pkg_share, 'urdf', 'jetrover_sim.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'sim_arena.world')
    model_path = os.path.join(pkg_share, 'models')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_sim.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'gui': gui,
            'verbose': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'jetrover',
            '-timeout', '90.0',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05',
            '-Y', yaw,
        ],
    )

    # 复用仓库的 slam_toolbox 配置（config/slam.yaml：scan/base_footprint/map/odom）
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam'),
                         'launch', 'include', 'slam_base.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            model_path + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', ''),
        ),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='启动 gazebo 客户端窗口'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='启动 RViz2 地图窗口'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo Classic world 文件',
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        slam,
        rviz,
    ])
