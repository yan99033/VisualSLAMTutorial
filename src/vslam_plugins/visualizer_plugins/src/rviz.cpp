#include "vslam_visualizer_plugins/rviz.hpp"

namespace vslam_visualizer_plugins {
  RViz::~RViz() {}

  void RViz::initialize() {}

  void RViz::add_live_frame(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::add_keyframe(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::replace_all_keyframes(const FrameVec& frame_msgs) {}

}  // namespace vslam_visualizer_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_visualizer_plugins::RViz, vslam_visualizer_base::Visualizer)