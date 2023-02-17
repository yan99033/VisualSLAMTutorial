# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vslam_demos'),
        'params_file',
        'test_kitti.yaml'
        )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='vslam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='data_loader_nodes',
                    plugin='vslam_components::data_loader_nodes::LoadFromFolder',
                    name='camera_node', 
                    parameters = [config],
                    extra_arguments=[{'use_intra_process_comms': True}],),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
