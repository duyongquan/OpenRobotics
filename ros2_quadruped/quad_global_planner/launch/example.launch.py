'''
Author: duyongquan duyongquan1@xiaomi.com
Date: 2023-07-28 21:06:20
LastEditors: duyongquan duyongquan1@xiaomi.com
LastEditTime: 2023-07-28 21:14:41
FilePath: /ros2_tutorials/src/ROS2-OpenRoboticsToolkit/ros2_quadruped/quad_global_planner/launch/example.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file = os.path.join(
        get_package_share_directory('quad_global_planner'),
        'params', 'params.yaml')


    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
            
    start_terrain_map_publisher_cmd = Node(
        package='quad_global_planner',
        executable='terrain_map_publisher_node',
        parameters = [params_file],
        output='screen',
        # prefix=['xterm -e gdb  --args'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the global planner nodes
    ld.add_action(start_terrain_map_publisher_cmd)

    return ld
