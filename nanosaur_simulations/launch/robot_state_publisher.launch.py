# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

try:
    from dotenv import load_dotenv, dotenv_values
except:
    print("Skip load dotenv library")


def generate_launch_description():
    nanosaur_simulations = get_package_share_directory('nanosaur_simulations')

    # Force load /opt/nanosaur/.env file
    # https://pypi.org/project/python-dotenv/
    try:
        load_dotenv('/opt/nanosaur/.env', override=True)
    except:
        print("Skip load .env variables")

    cover_type_conf = os.getenv("NANOSAUR_COVER_TYPE", 'fisheye')
    print(f"Load cover_type from ENV: {cover_type_conf}")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cover_type = LaunchConfiguration('cover_type')
    namespace = LaunchConfiguration('namespace', default="nanosaur")
    
    # Add option to publish pointcloud
    publish_pointcloud="False"
    publish_odom_tf="False"

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value=cover_type_conf,
        description='Cover type to use. Options: pi, fisheye, realsense, zed.')

    # full  path to urdf and world file
    # world = os.path.join(nanosaur_simulations, "worlds", world_file_name)
    xacro_path = os.path.join(nanosaur_simulations, "urdf", "nanosaur.urdf.xacro")

    # Launch Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(
                         [
                             'xacro ', xacro_path, ' ',
                             'robot_name:=', namespace, ' ',
                             'cover_type:=', cover_type, ' ',
                             'publish_pointcloud:=', publish_pointcloud, ' ',
                             'publish_odom_tf:=', publish_odom_tf, ' ',
                         ])
                     }]
    )
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(nanosaur_cmd)
    ld.add_action(declare_cover_type_cmd)
    ld.add_action(robot_state_publisher_node)

    return ld
# EOF