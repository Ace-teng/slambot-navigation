import os

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    qos = LaunchConfiguration('qos').perform(context)
    database_path = LaunchConfiguration('database_path').perform(context)
    clear_db = LaunchConfiguration('clear_db', default='false').perform(context)

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        # Database this mapping session writes to. Set Mem/IncrementalMemory and
        # load it again for localization (see navigation/.../include/rtabmap.launch.py).
        'database_path': database_path,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        # RTAB-Map's parameters should be strings:
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin': '0.2',  # ignore laser scan points on the robot itself
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Grid/Sensor': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/depth_cam/rgb/image_raw'),
        ('rgb/camera_info', '/depth_cam/rgb/camera_info'),
        ('depth/image', '/depth_cam/depth/image_raw'),
        ('odom', '/odom'),
    ]

    # Only a brand-new map should delete the database. Without clear_db:=true
    # the previous database at database_path is preserved (append / re-localize).
    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'] if clear_db == 'true' else [],
    )

    sync_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,
            'use_sim_time': use_sim_time,
            'qos': qos,
        }],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/rtabmap.db'),
            description='RTAB-Map database that this mapping session writes to. '
                        'Reuse the same path later for localization.'),
        DeclareLaunchArgument(
            'clear_db', default_value='false',
            description='Delete the database at database_path on start to begin a '
                        'brand-new map. Default preserves the previous database.'),
        sync_node,
        rtabmap_node,
    ])


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
