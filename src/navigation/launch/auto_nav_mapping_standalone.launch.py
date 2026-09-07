import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, OpaqueFunction, TimerAction)
from launch_ros.actions import PushRosNamespace


def launch_setup(context):
    # 真车「独立栈」自动导航建图（slam.launch 思路 + Nav2）。
    # 运行前提：先 stop_ros 杀掉开机 bringup（本文件自带 robot.launch 硬件，勿与 bringup 并存）。
    # 与叠加版(auto_nav_mapping.launch.py, 挂在 bringup 上)不同：硬件与 slam 在同一 launch 内自拉起，
    # 实测这样 rmw_zrdds 下 slam 才拿得到 TF、能出图。本文件为新增，不改动任何既有文件。
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

    enable_save = LaunchConfiguration('enable_save', default='false').perform(context)
    sim = LaunchConfiguration('sim', default='false').perform(context)
    use_joy = LaunchConfiguration('use_joy', default='false').perform(context)  # 自动导航禁摇杆
    use_teb = LaunchConfiguration('use_teb', default='false').perform(context)
    robot_name = LaunchConfiguration(
        'robot_name', default=os.environ.get('HOST', '/')).perform(context)
    master_name = LaunchConfiguration(
        'master_name', default=os.environ.get('MASTER', '/')).perform(context)

    enable_save_arg = DeclareLaunchArgument('enable_save', default_value=enable_save)
    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    use_joy_arg = DeclareLaunchArgument('use_joy', default_value=use_joy)
    use_teb_arg = DeclareLaunchArgument('use_teb', default_value=use_teb)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    frame_prefix = '' if robot_name == '/' else '%s/' % robot_name
    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    map_frame = '{}map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    scan_topic = '{}scan_raw'.format(frame_prefix)

    # 控制器参数：与 navigation.launch.py 一致地按机型选
    machine_type = os.environ.get('MACHINE_TYPE', 'LanderPi_Mecanum')
    if machine_type == 'LanderPi_Acker':
        params_file = os.path.join(navigation_package_path, 'config', 'nav2_params_ack.yaml')
    else:
        # 用我们专用的 nav2_params_auto.yaml（车体半径0.12/膨胀0.25/限速0.22·2.0，防贴墙与拐弯扫挂）
        params_file = os.path.join(navigation_package_path, 'config', 'nav2_params_auto.yaml')

    # 0) 硬件（controller/雷达/遥控开关），与 slam.launch 相同来源
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'use_joy': use_joy,
        }.items(),
    )

    # 1) slam_toolbox 在线建图（scan_topic/enable_save 照 slam.launch 传法）
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/slam_base.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'scan_topic': scan_topic,
            'enable_save': enable_save,
        }.items(),
    )

    # 2) Nav2：rtabmap='true' 跳过 AMCL/map_server，/map 吃 slam 在线地图（不开 rtabmap）
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
            'use_teb': use_teb,
        }.items(),
    )

    stack = GroupAction(actions=[
        PushRosNamespace(robot_name),
        base_launch,
        TimerAction(period=10.0, actions=[slam_launch]),
        TimerAction(period=15.0, actions=[navigation_launch]),
    ])

    return [enable_save_arg, sim_arg, use_joy_arg, use_teb_arg,
            master_name_arg, robot_name_arg, stack]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
