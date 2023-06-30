import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_demos')
    launch_dir = os.path.join(bringup_dir, 'launch')
    maps_dir = os.path.join(bringup_dir, 'maps')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    start_a_star_planner_cmd = Node(
        package='nav2_demos',
        executable='tutorial.nav2.a_star_planner',
        name='a_star_planner',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
         # prefix=['xterm -e gdb  --args'],)

     # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)
    ld.add_action(start_a_star_planner_cmd)

    return ld
