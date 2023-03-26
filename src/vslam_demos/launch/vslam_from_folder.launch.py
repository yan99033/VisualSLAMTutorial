# TODO: licence

"""Launch the vslam components on images in a folder"""

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    default_config = os.path.join(
        get_package_share_directory('vslam_demos'),
        'config',
        'test_kitti.yaml'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    config = LaunchConfiguration('config')

    """Generate launch description with multiple components."""
    vslam_preprocessing_container = ComposableNodeContainer(
        name='vslam_preprocessing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='data_loader_nodes',
                plugin='vslam_components::data_loader_nodes::LoadFromFolder',
                name='camera_node',
                parameters=[config],
                remappings=[
                    ('/out_frame', '/raw_frame'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],),
            ComposableNode(
                package='vslam_nodes',
                plugin='vslam_components::vslam_nodes::IndirectVSlamNode',
                name='indirect_vslam_node',
                parameters=[config],
                remappings=[
                    ('/in_frame', '/raw_frame'),
                    ('/out_frame', '/frame_w_features'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],),
            ComposableNode(
                package='visualization_nodes',
                plugin='vslam_components::visualization_nodes::RvizVisualNode',
                name='rviz_visual_node',
                remappings=[
                    ('/in_frame', '/frame_w_features'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],),
        ],
        output='screen',
    )

    # vslam_frontend_nodes
    return launch.LaunchDescription([declare_config_file_cmd, vslam_preprocessing_container])
