#include "vslam_nodes/vslam_node.hpp"

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
}  // namespace

namespace vslam_components {

  namespace vslam_nodes {

    VSlamNode::VSlamNode(const rclcpp::NodeOptions &options) : Node("vslam_node", options) {
      // ORB feature detector
      orb_feature_detector_ = cv::ORB::create(declare_parameter("num_features", 2000));

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&VSlamNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    void VSlamNode::frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) const {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Getting frame %u\n", frame_msg->id);

      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width,
                     encoding2mat_type(frame_msg->image.encoding), frame_msg->image.data.data());

      // Extract features in the image

      pub_ptr->publish(std::move(frame_msg));
    }
  }  // namespace vslam_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::vslam_nodes::VSlamNode)