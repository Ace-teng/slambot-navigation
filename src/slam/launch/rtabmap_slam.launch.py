import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)


def launch_setup(context):
    compiled = os.environ['need_compile']
    sim = LaunchConfiguration('sim', default='false').perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    use_sim_time = 'true' if sim == 'true' else 'false'

    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'action_name': 'horizontal',
        }.items(),
    )

    # 3D (RGB-D + LiDAR) mapping is done by RTAB-Map only.
    # NOTE: slam_toolbox (2D) must NOT be launched here: it would publish its own
    # /map topic and map->odom TF and fight RTAB-Map over the same transform tree.
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bringup_launch = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            base_launch,
            TimerAction(
                period=10.0,  # 延时等待其它节点启动好
                actions=[rtabmap_launch],
            ),
        ],
    )

    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
