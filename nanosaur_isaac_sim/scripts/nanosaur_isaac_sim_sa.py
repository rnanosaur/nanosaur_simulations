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

import sys
import carb
from omni.isaac.kit import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)

import omni.graph.core as og
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import nucleus
from omni.kit import commands
from omni import usd
from omni.isaac.core_nodes.scripts.utils import set_target_prims


# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

# Locate assets root folder to load sample
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

PATH_LOCAL_URDF_FOLDER="/tmp/robot.urdf"

class IsaacWorld():
    
    def __init__(self, stage_path=""):
        self.commands=[]
        # Setting up scene
        if stage_path:
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            usd_path = assets_root_path + stage_path
            usd.get_context().open_stage(usd_path, None)
        else:
            self.simulation_context = World(stage_units_in_meters=1.0)
            self.simulation_context.scene.add_default_ground_plane()
        # need to initialize physics getting any articulation..etc
        self.simulation_context.initialize_physics()
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
    
    def add_tick(self, function):
        self.commands += [function]
    
    def wait_step_reload(self):
        self.simulation_context.step(render=True)
        print("Loading stage...")
        while is_stage_loading():
            simulation_app.update()
        print("Loading Complete")

    def start_simulation(self):
        self.simulation_context.play()

    def run_simulation(self, node):
        while simulation_app.is_running():
            self.simulation_context.step(render=True)
            # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
            for command in self.commands:
                command()
            rclpy.spin_once(node, timeout_sec=0.0)
            if self.simulation_context.is_playing():
                if self.simulation_context.current_time_step_index == 0:
                    self.simulation_context.reset()
        # Cleanup
        self.simulation_context.stop()
        simulation_app.close()


class RobotLoader(Node):
    
    def __init__(self, isaac_world, namespace="nanosaur"):
        super().__init__("robot_loader", namespace=namespace)
        # Load isaac_world
        self.namespace = namespace
        self.isaac_world = isaac_world
        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(String, "robot_description", self.callback_description, 1)
        self.ros_sub  # prevent unused variable warning
        # Node started
        self.get_logger().info("Robot loader start")

    def load_robot(self, robot_name):
        # Setting up import configuration:
        status, import_config = commands.execute(
            "URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = False
        import_config.fix_base = False
        import_config.distance_scale = 1

        # Import URDF, stage_path contains the path the path to the usd prim in the stage.
        # extension_path = get_extension_path_from_name("omni.isaac.urdf")
        status, stage_path = commands.execute(
            "URDFParseAndImportFile",
            urdf_path=PATH_LOCAL_URDF_FOLDER,
            import_config=import_config,
        )
        # Wait a step
        self.isaac_world.wait_step_reload()
        
        # Creating a action graph with ROS component nodes
        # https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_python.html
        # https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_turtlebot.html#build-the-graph
        try:
            og.Controller.edit(
                {"graph_path": f"/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("ROS2SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("scale_to_from_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                        ("break_3_vector_01", "omni.graph.nodes.BreakVector3"),
                        ("break_3_vector_02", "omni.graph.nodes.BreakVector3"),
                        ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                        ("ConstantToken_01", "omni.graph.nodes.ConstantToken"),
                        ("ConstantToken_02", "omni.graph.nodes.ConstantToken"),
                        ("MakeArray", "omni.graph.nodes.MakeArray"),
                        ("IsaacArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ROS2SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                        ("ROS2SubscribeTwist.outputs:angularVelocity", "break_3_vector_01.inputs:tuple"),
                        ("ROS2SubscribeTwist.outputs:linearVelocity", "scale_to_from_stage_units.inputs:value"),
                        ("scale_to_from_stage_units.outputs:result", "break_3_vector_02.inputs:tuple"),
                        ("ROS2SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                        ("break_3_vector_01.outputs:z", "DifferentialController.inputs:angularVelocity"),
                        ("break_3_vector_02.outputs:x", "DifferentialController.inputs:linearVelocity"),
                        ("OnPlaybackTick.outputs:tick", "IsaacArticulationController.inputs:execIn"),
                        ("DifferentialController.outputs:velocityCommand", "IsaacArticulationController.inputs:velocityCommand"),
                        ("ConstantToken_01.inputs:value", "MakeArray.inputs:a"),
                        ("ConstantToken_02.inputs:value", "MakeArray.inputs:b"),
                        ("MakeArray.outputs:array", "IsaacArticulationController.inputs:jointNames"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Assigning a Domain ID of 1 to Context node
                        ("ROS2Context.inputs:domain_id", 0),
                        # Assigning topic name to clock publisher
                        ("ROS2SubscribeTwist.inputs:topicName", "/nanosaur/cmd_vel"),
                        # Assigning Differential controller configuration
                        ("DifferentialController.inputs:maxLinearSpeed", 0.5),
                        ("DifferentialController.inputs:wheelDistance", 0.4*2),
                        ("DifferentialController.inputs:wheelRadius", 0.0150),
                        # Assign Articulation controller configuration
                        ("IsaacArticulationController.inputs:usePath", False),
                        # Assigning topic name to clock publisher
                        ("ConstantToken_01.inputs:value", "sprocket_left_joint"),
                        ("ConstantToken_02.inputs:value", "sprocket_right_joint"),
                    ]
                },
            )
        except Exception as e:
            print(e)
        
        NANOSAUR_STAGE_PATH="/nanosaur/base_link"
        # Setting the /Franka target prim to Subscribe JointState node
        set_target_prims(primPath="/ActionGraph/IsaacArticulationController", targetPrimPaths=[NANOSAUR_STAGE_PATH])

    def callback_description(self, msg):
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        print(f"Load robot")
        robot_urdf = msg.data
        #print(robot_urdf)
        robot_name = "nanosaur"
        text_file = open(PATH_LOCAL_URDF_FOLDER, "w")
        n = text_file.write(robot_urdf)
        text_file.close()
        # Load robot
        self.load_robot(robot_name)


if __name__ == "__main__":
    rclpy.init()
    # Isaac SIM world
    isaac_world = IsaacWorld()
    # Start simulation
    isaac_world.start_simulation()
    # Initialize robot loader
    robot_loader = RobotLoader(isaac_world)
    # Run simulation
    isaac_world.run_simulation(robot_loader)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_loader.destroy_node()
    rclpy.shutdown()
# EOF