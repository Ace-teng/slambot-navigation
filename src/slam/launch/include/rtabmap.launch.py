from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          # Database this mapping session writes to. Set Mem/IncrementalMemory and
          # load it again for localization (see navigation/.../include/rtabmap.launch.py).
          'database_path':database_path,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Grid/Sensor': 'true',
          'RGBD/ProximityPathMaxNeighbors': '10',

    }

    remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('rgb/image', '/depth_cam/rgb/image_raw'),
            ('rgb/camera_info', '/depth_cam/rgb/camera_info'),
            ('depth/image', '/depth_cam/depth/image_raw'),
            ('odom', '/odom'),
          ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database that this mapping session writes to. '
                        'Reuse the same path later for localization.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'use_sim_time': use_sim_time,
                'qos': qos,
            }],
            remappings=remappings),

        # SLAM Mode: '-d' clears the database on startup -> start a brand new map.
        # The map is persisted to 'database_path' (or explicitly saved from rtabmapviz
        # / with the map_saver action); load it in localization mode to relocalize.
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
