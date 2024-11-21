import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    demo_dir = get_package_share_directory('demo_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(demo_dir, 'map', 'jrnc.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(demo_dir, 'config', 'nav2_param.yaml'))
    rviz_config_dir = os.path.join(demo_dir, 'rviz', 'nav2.rviz')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value = 'False',
        description   = 'Whether run a SLAM'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
        'map': map_yaml_path,
        'use_sim_time': use_sim_time,
        'params_file': nav2_param_path}.items()
    )
    
    rviz_node =  Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        arguments  = ['-d', rviz_config_dir],
        parameters = [{'use_sim_time': use_sim_time}],
        # output     = 'screen'
    )
    
    return LaunchDescription([
        declare_use_slam_cmd, 
        nav2_bringup_launch, 
        rviz_node
    ])
