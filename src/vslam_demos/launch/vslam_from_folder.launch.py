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
                parameters=[config],
                extra_arguments=[{'use_intra_process_comms': True}],),
    ],
    output='screen',
  )

  return launch.LaunchDescription([declare_config_file_cmd, container])
