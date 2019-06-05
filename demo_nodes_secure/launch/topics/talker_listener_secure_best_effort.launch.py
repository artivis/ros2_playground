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

"""Launch a secured talker and a secured best effort listener."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
import launch_ros.actions

import os
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    ros_security_root_directory = os.path.join(get_package_prefix('demo_nodes_secure'), 'ros2_security', 'keystore')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_SECURITY_ENABLE', 'true'),
        SetEnvironmentVariable('ROS_SECURITY_STRATEGY', 'Enforce'),
        # TODO: ROS_SECURITY_NODE_DIRECTORY crashes
        SetEnvironmentVariable('ROS_SECURITY_ROOT_DIRECTORY', ros_security_root_directory),
        launch_ros.actions.Node(
            package='demo_nodes_secure', node_executable='talker', output='screen'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='listener_best_effort', output='screen'),
    ])
