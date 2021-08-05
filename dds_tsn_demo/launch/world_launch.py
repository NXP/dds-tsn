# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch a world model in Gazebo"""

from launch import LaunchDescription
import launch_ros.actions
import launch.actions
import os
import sys

def generate_launch_description():
    gazebo_world_path = os.path.join(os.getcwd(), 'dds_tsn_demo/world')
    gazebo_world = os.path.join(gazebo_world_path, 'gazebo_diff_drive_moose_test.world')
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world],
            additional_env={'GAZEBO_MODEL_PATH': gazebo_world_path})])
