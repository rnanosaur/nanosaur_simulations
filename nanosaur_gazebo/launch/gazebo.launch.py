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
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


class Coordinate:

    def safe_list_get(self, l, idx, default=0.0):
        try:
            return str(l[idx])
        except IndexError:
            return str(default)

    def __init__(self, config={}) -> None:
        position = config.get('xyz', [])
        orientation = config.get('RPY', [])
        self.x = self.safe_list_get(position, 0)
        self.y = self.safe_list_get(position, 1)
        self.z = self.safe_list_get(position, 2)
        self.R = self.safe_list_get(orientation, 0)
        self.P = self.safe_list_get(orientation, 1)
        self.Y = self.safe_list_get(orientation, 2)
        
    def __repr__(self) -> str:
        coordinate = f"xyz=[{self.x} {self.y} {self.z}] RPY=[{self.R} {self.P} {self.Y}]"
        return coordinate


def load_robot_position(config, world_file_name):
    # Extract worldfile name from configuration
    world_name = Path(world_file_name).stem
    # Check fi file exist
    if not os.path.isfile(config):
        print("no file available")
        return Coordinate()
    # Load yml file
    with open(config, "r") as stream:
        try:
            robot_config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return Coordinate()
    # Check if world exist
    if world_name not in robot_config:
        return Coordinate()
    # load position and orientation
    config = robot_config[world_name]
    # Extract configuration 
    return Coordinate(config)


def launch_gazebo_setup(context: LaunchContext, support_namespace, support_world):
    """ Reference:
        https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ 
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
    """
    package_worlds = get_package_share_directory('nanosaur_worlds')
    # render namespace, dumping the support_package.
    namespace = context.perform_substitution(support_namespace)
    world_name = context.perform_substitution(support_world)
    print(f"Loading world: {world_name}")
    # Load configuration from params
    conf = load_robot_position(os.path.join(package_worlds, 'params', 'spawn_robot.yml'), world_name)
    # Spawn robot
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/scripts/spawn_entity.py
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        namespace=namespace,
        arguments=['-entity', 'nanosaur',
                   '-topic', 'robot_description',
                   '-x', conf.x, '-y', conf.y, '-z',conf.z,
                   '-R', conf.R, '-P', conf.P, '-Y',conf.Y,
                   ]
    )

    return [spawn_robot]


def generate_launch_description():
    package_gazebo = get_package_share_directory('nanosaur_gazebo')
    package_worlds = get_package_share_directory('nanosaur_worlds')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    pkg_control = get_package_share_directory('nanosaur_control')
    
    default_world_name = 'office.world' # Empty world: empty_world.world
    
    launch_file_dir = os.path.join(package_gazebo, 'launch')

    rviz = LaunchConfiguration('rviz')
    world_file_name = LaunchConfiguration('world_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default="nanosaur")

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    rviz_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='True',
        description='Flag to enable rviz visualization')

    gazebo_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to "false" to run headless.')

    gazebo_server_cmd = DeclareLaunchArgument(
        name='server',
        default_value='true',
        description='Set to "false" not to run gzserver.')

    world_file_name_cmd = DeclareLaunchArgument(
        name='world_file_name',
        default_value=default_world_name,
        description='Load gazebo world.')

    # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
    # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
    # Reference options
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/launch/gzserver.launch.py
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_path, 'launch'), '/gzserver.launch.py']),
        launch_arguments={'world': [package_worlds, "/worlds/", world_file_name],
                          'verbose': 'true',
                          'init': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('server'))
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_path, 'launch'), '/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    rsp_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz)
    )

    ###################### Twist controls ######################

    # include another launch file in nanosaur namespace
    twist_control_launch = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            # nanosaur twist launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_control, '/launch/twist_control.launch.py']))
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(nanosaur_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(gazebo_gui_cmd)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(world_file_name_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_gui)
    ld.add_action(rsp_launcher)
    ld.add_action(OpaqueFunction(function=launch_gazebo_setup, args=[namespace]))
    ld.add_action(twist_control_launch)
    ld.add_action(rviz2)

    return ld
# EOF
