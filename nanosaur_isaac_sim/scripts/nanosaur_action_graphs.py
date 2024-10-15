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


def build_mecanum_controller_graph(robot_name):   
    # Creating a action graph with ROS component nodes
    try:
        (_, _, _, _) = og.Controller.edit(
            {
                "graph_path": f"/{robot_name}/ActionGraph",
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [],
                og.Controller.Keys.CONNECT: [],
                og.Controller.Keys.SET_VALUES: []
            },
        )
    except Exception as e:
        print(e)


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
