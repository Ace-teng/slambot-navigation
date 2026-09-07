from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)

    # LD14P节点
    ld14p_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD14P',
        output='screen',
        parameters=[
            {'topic_name': 'scan',
            'product_name': 'LDLiDAR_LD14P',
            'port_baudrate': 115200,
            'port_name': '/dev/lidar',
            'frame_id': lidar_frame,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0}
        ],
        remappings=[('scan', scan_raw)]
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        ld14p_node,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
