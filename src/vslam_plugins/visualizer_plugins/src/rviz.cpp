#include "vslam_visualizer_plugins/rviz.hpp"

namespace vslam_visualizer_plugins {
  RViz::~RViz() {}

  void RViz::initialize() {
    node_ = std::make_shared<rclcpp::Node>("visualizer_node", "vslam");

    image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("live_image", 1);
    live_frame_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("live_frame_marker", 10);
    keyframe_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("keyframe_marker", 10);
  }

  void RViz::add_live_frame(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::add_keyframe(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::replace_all_keyframes(const FrameVec& frame_msgs) {}

}  // namespace vslam_visualizer_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_visualizer_plugins::RViz, vslam_visualizer_base::Visualizer)