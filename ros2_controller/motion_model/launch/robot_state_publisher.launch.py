#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Robot urdf file
    diff_urdf = os.path.join(
        get_package_share_directory('motion_model'),
        'urdf',
        'diff',
        'differential_bot.urdf.xacro'
    )

    bicycle_urdf = os.path.join(
        get_package_share_directory('motion_model'),
        'urdf',
        'bicycle',
        'bicycle_bot.urdf.xacro'
    )

    ackermann_urdf = os.path.join(
        get_package_share_directory('motion_model'),
        'urdf',
        'ackermann',
        'ackermann.urdf.xacro'
    )

    holonomic__urdf = os.path.join(
        get_package_share_directory('motion_model'),
        'urdf',
        'holonomic',
        'holonomic.urdf.xacro'
    )

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = LaunchConfiguration('urdf_file_name', default='')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    print('urdf_file_name : {}'.format(ackermann_urdf))


    robot_state_publisher_cmd = Node(package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'urdf_file_name': urdf_file_name,
            'robot_description': ParameterValue(Command(['xacro ', str(ackermann_urdf)]), value_type=str)
        }],
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
