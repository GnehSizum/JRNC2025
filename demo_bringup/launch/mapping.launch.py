import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    demo_dir = get_package_share_directory('demo_bringup')
    slam_toolbox_mapping_file_dir = os.path.join(demo_dir, 'config', 'mapper_params_online_async_sim.yaml')
    rviz_config_dir = os.path.join(demo_dir, 'rviz', 'mapping.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 

    start_mapping = Node(
        package    = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name       = 'slam_toolbox',
        parameters = [
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ])

    rviz_node =  Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        arguments  = ['-d', rviz_config_dir],
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen')
    
    return LaunchDescription([
        start_mapping, 
        rviz_node
        ])
