
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
# EOF
