#include "feature_matching_nodes/orb_matcher_node.hpp"

using std::placeholders::_1;

namespace vslam_components {
  namespace feature_matching_nodes {

    OrbMatcherNode::OrbMatcherNode(const rclcpp::NodeOptions &options)
        : Node("orb_matcher_node", options) {
      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&OrbMatcherNode::frame_matching_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    void OrbMatcherNode::frame_matching_callback(
        vslam_msgs::msg::Frame::UniquePtr frame_msg) const {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "OrbMatcherNode: unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "OrbMatcherNode: Received frame: %u", frame_msg->id);
    }

  }  // namespace feature_matching_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::feature_matching_nodes::OrbMatcherNode)