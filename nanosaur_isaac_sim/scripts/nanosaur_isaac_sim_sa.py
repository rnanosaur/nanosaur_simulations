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


from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {"renderer": "RayTracedLighting", "headless": False})

from pxr import Sdf, Gf, UsdPhysics, UsdLux, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core import World
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core.utils.extensions import enable_extension, get_extension_path_from_name
import omni

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

PATH_LOCAL_URDF_FOLDER="/tmp/robot.urdf"

class IsaacWorld():
    
    def __init__(self):
        # setting up the world
        self.timeline = omni.timeline.get_timeline_interface()
        # Setting up scene
        self.setup_scene()
        
    def setup_scene(self):
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()
        # Get stage handle
        self.stage = omni.usd.get_context().get_stage()
        # Enable physics
        scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
        # Set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        # Set solver settings
        PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath("/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self.stage, "/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")
        # reset world
        self.ros_world.reset()

    def get_stage(self):
        return self.stage
    
    def wait_step_reload(self):
        self.ros_world.step(render=True)
        print("Loading stage...")
        while is_stage_loading():
            simulation_app.update()
        print("Loading Complete")

    def start_simulation(self):
        self.timeline.play()

    def run_simulation(self, node):
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 0:
                    self.ros_world.reset()
        # Cleanup
        self.timeline.stop()
        simulation_app.close()


class RobotLoader(Node):
    
    def __init__(self, isaac_world, topic_name="/nanosaur/robot_description"):
        super().__init__("robot_loader")
        # Load isaac_world
        self.isaac_world = isaac_world
        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(String, topic_name, self.callback_description, 1)
        self.ros_sub  # prevent unused variable warning
        # Node started
        self.get_logger().info("Robot loader start")

    def load_robot(self):
        # Setting up import configuration:
        status, import_config = omni.kit.commands.execute(
            "URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = False
        import_config.fix_base = False
        import_config.distance_scale = 1

        # Import URDF, stage_path contains the path the path to the usd prim in the stage.
        # extension_path = get_extension_path_from_name("omni.isaac.urdf")
        status, stage_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=PATH_LOCAL_URDF_FOLDER,
            # urdf_path=extension_path + "/data/urdf/robots/carter/urdf/carter.urdf",
            import_config=import_config,
        )
        # Wait a step
        self.isaac_world.wait_step_reload()
        # Get Isaac SIM world stage
        stage = self.isaac_world.get_stage()
        # Get handle to the Drive API for both wheels
        left_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/nanosaur/base_link/sprocket_left_joint"), "angular")
        right_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/nanosaur/base_link/sprocket_right_joint"), "angular")

        # Get handle to the Drive API for both wheels
        #left_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/left_wheel"), "angular")
        #right_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/right_wheel"), "angular")

        # Set the velocity drive target in degrees/second
        left_wheel_drive.GetTargetVelocityAttr().Set(150)
        right_wheel_drive.GetTargetVelocityAttr().Set(150)

        # Set the drive damping, which controls the strength of the velocity drive
        left_wheel_drive.GetDampingAttr().Set(15000)
        right_wheel_drive.GetDampingAttr().Set(15000)

        # Set the drive stiffness, which controls the strength of the position drive
        # In this case because we want to do velocity control this should be set to zero
        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)

        # dynamic control can also be used to interact with the imported urdf.
        dc = _dynamic_control.acquire_dynamic_control_interface()

        art = dc.get_articulation(stage_path)

        if art == _dynamic_control.INVALID_HANDLE:
            print(f"{stage_path} is not an articulation")
        else:
            print(f"Got articulation {stage_path} with handle {art}")

    def callback_description(self, msg):
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        print(f"Load robot")
        robot_urdf = msg.data
        text_file = open(PATH_LOCAL_URDF_FOLDER, "w")
        n = text_file.write(robot_urdf)
        text_file.close()
        # Load robot
        self.load_robot()


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