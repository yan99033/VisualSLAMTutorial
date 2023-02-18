#include "feature_extraction_nodes/fast_feature_extraction_node.hpp"

using std::placeholders::_1;

namespace vslam_components {

  namespace feature_extraction_nodes {

    FastFeatureExtractionNode::FastFeatureExtractionNode(const rclcpp::NodeOptions &options)
        : Node("fast_feature_extraction_node", options) {
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
        "in_frame", 10, std::bind(&FastFeatureExtractionNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
    }

    void FastFeatureExtractionNode::frame_callback(const vslam_msgs::msg::Frame::SharedPtr frame_msg) const {
      RCLCPP_INFO(this->get_logger(), "Getting frame %u\n", frame_msg->id);
    }
  } // namespace vslam_components
} // namespace feature_extraction_nodes

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::feature_extraction_nodes::FastFeatureExtractionNode)