#include "visualization_nodes/rviz_visual_node.hpp"

using std::placeholders::_1;

namespace {
int encoding2mat_type(const std::string &encoding) {
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}
} // namespace

namespace vslam_components {

namespace visualization_nodes {

RvizVisualNode::RvizVisualNode(const rclcpp::NodeOptions &options)
    : Node("rviz_visual_node", options) {
  // Frame subscriber and publisher
  frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
      "in_frame", 10, std::bind(&RvizVisualNode::frame_callback, this, _1));
}

void RvizVisualNode::frame_callback(
    vslam_msgs::msg::Frame::UniquePtr frame_msg) {
  RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);
}
} // namespace visualization_nodes
} // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    vslam_components::visualization_nodes::RvizVisualNode)