import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        declare_rviz_config_file_cmd = DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                rviz, 'rviz', 'demo1_view.rviz'),
            description='Full path to the RVIZ config file to use')

        Node(
            package='rviz',
            executable='tutorial.rviz.basic_shapes',
            output='screen',
            # prefix=['xterm -e gdb  --args'],
        )
    ])
