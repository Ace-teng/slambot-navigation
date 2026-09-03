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
          # Database that was built by a mapping session
          # (slam/.../include/rtabmap.launch.py). Loaded here for relocalization.
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
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
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
            description='RTAB-Map database created by the mapping session. '
                        'Loaded here to relocalize the robot inside it.'),

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

        # Localization mode: disable mapping (Mem/IncrementalMemory=False) and load
        # the whole map into working memory, so the robot relocalizes against the
        # database built by the mapping session (see slam/.../include/rtabmap.launch.py).
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
