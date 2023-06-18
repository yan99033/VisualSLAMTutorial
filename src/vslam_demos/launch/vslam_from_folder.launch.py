#  This file is part of VisualSLAMTutorial
#
#  Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""Launch the vslam components on images in a folder"""

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    default_params = os.path.join(
        get_package_share_directory('vslam_demos'),
        'params',
        'test_kitti.yaml'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params', default_value=default_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    params = LaunchConfiguration('params')

    """Generate launch description with multiple components."""
    vslam_preprocessing_container = ComposableNodeContainer(
        name='vslam_preprocessing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='vslam_nodes',
                plugin='vslam_components::vslam_nodes::IndirectVSlamNode',
                name='indirect_vslam_node',
                parameters=[params],
                remappings=[
                    ('/in_frame', '/raw_frame'),
                    ('/out_frame', '/live_frame'),
                    ('/out_keyframe', '/keyframe')
                ]),
            ComposableNode(
                package='data_loader_nodes',
                plugin='vslam_components::data_loader_nodes::DataLoaderNode',
                name='camera_node',
                parameters=[params],
                remappings=[
                    ('/out_frame', '/raw_frame'),
                ])
        ],
        output='screen',
    )

    default_rviz_config = os.path.join(
        get_package_share_directory('vslam_demos'),
        'config',
        'rviz_config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
    )

    # vslam_frontend_nodes
    return launch.LaunchDescription([declare_params_file_cmd, vslam_preprocessing_container, rviz_node])
