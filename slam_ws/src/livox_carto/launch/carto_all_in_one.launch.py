from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   cloud_topic     = LaunchConfiguration('cloud_topic')
   scan_topic      = LaunchConfiguration('scan_topic')
   target_frame    = LaunchConfiguration('target_frame')
   min_height      = LaunchConfiguration('min_height')
   max_height      = LaunchConfiguration('max_height')
   angle_increment = LaunchConfiguration('angle_increment')
   range_min       = LaunchConfiguration('range_min')
   range_max       = LaunchConfiguration('range_max')
   carto_dir       = LaunchConfiguration('carto_dir')
   carto_file      = LaunchConfiguration('carto_file')

   return LaunchDescription([
       DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
       DeclareLaunchArgument('scan_topic',  default_value='/scan'),
       DeclareLaunchArgument('target_frame', default_value='livox_frame'),
       DeclareLaunchArgument('min_height',   default_value='-1.0'),
       DeclareLaunchArgument('max_height',   default_value='1.0'),
       DeclareLaunchArgument('angle_increment', default_value='0.0087266'),
       DeclareLaunchArgument('range_min',    default_value='0.2'),
       DeclareLaunchArgument('range_max',    default_value='30.0'),
       DeclareLaunchArgument(
           'carto_dir',
           default_value=PathJoinSubstitution([FindPackageShare('livox_carto'), 'config'])
       ),
       DeclareLaunchArgument('carto_file',   default_value='carto.lua'),

       # TF: odom -> livox_frame
       Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           name='static_tf_pub',
           arguments=[
               '--x','0','--y','0','--z','0',
               '--roll','0','--pitch','0','--yaw','0',
               '--frame-id','odom',
               '--child-frame-id', target_frame
           ],
           output='screen'
       ),

       # 点云 -> 激光
       Node(
           package='pointcloud_to_laserscan',
           executable='pointcloud_to_laserscan_node',
           name='pcl2scan',
           remappings=[('cloud_in', cloud_topic), ('scan', scan_topic)],
           parameters=[{
               'target_frame': target_frame,
               'min_height':   min_height,
               'max_height':   max_height,
               'angle_min':    -3.14159,
               'angle_max':     3.14159,
               'angle_increment': angle_increment,
               'range_min':    range_min,
               'range_max':    range_max,
               'use_inf':      True,

               # QoS 覆盖：订阅上游点云用 Best Effort，匹配 Livox
               'qos_overrides./cloud_in.subscription.reliability': 'best_effort',
               'qos_overrides./cloud_in.subscription.durability':  'volatile',
               'qos_overrides./cloud_in.subscription.history':     'keep_last',
               'qos_overrides./cloud_in.subscription.depth':       10,

               # QoS 覆盖：/scan 发布端也用 Best Effort（与下游一致）
               'qos_overrides./scan.publisher.reliability': 'best_effort',
               'qos_overrides./scan.publisher.durability':  'volatile',
               'qos_overrides./scan.publisher.history':     'keep_last',
               'qos_overrides./scan.publisher.depth':       10,
           }],
           output='screen'
       ),

       # Cartographer（用 arguments 传 lua 配置）
       Node(
           package='cartographer_ros',
           executable='cartographer_node',
           name='cartographer_node',
           output='screen',
           arguments=[
               '-configuration_directory', carto_dir,
               '-configuration_basename',  carto_file
           ],
           remappings=[('scan', scan_topic)],
           parameters=[{
               'qos_overrides./scan.subscription.reliability': 'best_effort',
               'qos_overrides./scan.subscription.durability':  'volatile',
               'qos_overrides./scan.subscription.history':     'keep_last',
               'qos_overrides./scan.subscription.depth':       10,
           }]
       ),

       # 发布 occupancy grid
       Node(
           package='cartographer_ros',
           executable='cartographer_occupancy_grid_node',
           name='occupancy_grid_node',
           parameters=[{'resolution': 0.05, 'publish_period_sec': 1.0}],
           output='screen'
       ),

       # 需要的话把下面解注一起开 RViz（先确保有 carto.rviz）
       # Node(
       #     package='rviz2',
       #     executable='rviz2',
       #     arguments=['-d', PathJoinSubstitution([FindPackageShare('livox_carto'), 'config', 'carto.rviz'])],
       #     output='screen'
       # ),
   ])
 

