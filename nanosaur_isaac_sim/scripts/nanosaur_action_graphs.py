# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
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

import omni
import omni.graph.core as og
from omni.isaac.core.utils import stage
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from pxr import Gf, UsdGeom
from omni.kit import commands
from pxr import Sdf, Usd
from omni.isaac.core.utils.prims import set_targets


def build_mecanum_controller_graph(robot_name, base_link_name="base_link"):
    mecanum_graph_name = f"/{robot_name}/ROS_HolonomicControllerGraph"
    # Creating a action graph with ROS component nodes
    try:
        (_, _, _, _) = og.Controller.edit(
            {
                "graph_path": mecanum_graph_name,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ("scale_to_from_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("break_3_vector_01", "omni.graph.nodes.BreakVector3"),
                    ("break_3_vector_02", "omni.graph.nodes.BreakVector3"),
                    ("Make3Vector", "omni.graph.nodes.MakeVector3"),
                    ("HolonomicController", "omni.isaac.wheeled_robots.HolonomicController"),
                    ("HolonomicRobotUsdSetup", "omni.isaac.wheeled_robots.HolonomicRobotUsdSetup"),
                    ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ROS2SubscribeTwist.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                    ("ROS2SubscribeTwist.outputs:angularVelocity", "break_3_vector_01.inputs:tuple"),
                    ("ROS2SubscribeTwist.outputs:linearVelocity", "scale_to_from_stage_units.inputs:value"),
                    ("scale_to_from_stage_units.outputs:result", "break_3_vector_02.inputs:tuple"),
                    ("break_3_vector_01.outputs:z", "Make3Vector.inputs:x"),
                    ("break_3_vector_02.outputs:x", "Make3Vector.inputs:z"),
                    ("ROS2SubscribeTwist.outputs:execOut", "HolonomicController.inputs:execIn"),
                    ("Make3Vector.outputs:tuple", "HolonomicController.inputs:inputVelocity"),
                    ("HolonomicRobotUsdSetup.outputs:mecanumAngles", "HolonomicController.inputs:mecanumAngles"),
                    ("HolonomicRobotUsdSetup.outputs:upAxis", "HolonomicController.inputs:upAxis"),
                    ("HolonomicRobotUsdSetup.outputs:wheelAxis", "HolonomicController.inputs:wheelAxis"),
                    ("HolonomicRobotUsdSetup.outputs:wheelOrientations", "HolonomicController.inputs:wheelOrientations"),
                    ("HolonomicRobotUsdSetup.outputs:wheelPositions", "HolonomicController.inputs:wheelPositions"),
                    ("HolonomicRobotUsdSetup.outputs:wheelRadius", "HolonomicController.inputs:wheelRadius"),
                    ("HolonomicRobotUsdSetup.outputs:wheelDofNames", "IsaacArticulationController.inputs:jointNames"),
                    ("HolonomicController.outputs:jointVelocityCommand", "IsaacArticulationController.inputs:velocityCommand"),
                    ],
                og.Controller.Keys.SET_VALUES: [
                    # Assigning a Domain ID of 1 to Context node
                    ("ROS2Context.inputs:domain_id", 0),
                    # Assigning topic name to clock publisher
                    ("ROS2SubscribeTwist.inputs:topicName", "/cmd_vel"),
                ]
            },
        )
    except Exception as e:
        print(e)
        
    robot_stage_path=f"/{robot_name}/{base_link_name}"
    # Setting the /Franka target prim to Subscribe JointState node
    set_target_prims(primPath=f"{mecanum_graph_name}/IsaacArticulationController", targetPrimPaths=[robot_stage_path])

def build_clock_graph():
    try:
        (_, _, _, _) = og.Controller.edit(
            {
                "graph_path": f"/Clock",
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishClock.inputs:timeStamp"),
                ]
            },
        )
    except Exception as e:
        print(e)
# EOF
