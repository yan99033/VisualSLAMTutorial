#include "feature_extraction_nodes/fast_feature_extraction_node.hpp"

#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

namespace vslam_components {

  namespace feature_extraction_nodes {

    FastFeatureExtractionNode::FastFeatureExtractionNode(const rclcpp::NodeOptions &options)
        : Node("fast_feature_extraction_node", options) {
      // FAST feature detector
      fast_feature_detector_ = cv::ORB::create(declare_parameter("num_features", 2000));

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&FastFeatureExtractionNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
    }

    void FastFeatureExtractionNode::frame_callback(
        const vslam_msgs::msg::Frame::SharedPtr frame_msg) const {
      RCLCPP_INFO(this->get_logger(), "FastFeatureExtractionNode: Getting frame %u\n",
                  frame_msg->id);

      // Get the image in the frame
      cv_bridge::CvImagePtr cv_ptr
          = cv_bridge::toCvCopy(frame_msg->image, frame_msg->image.encoding);

      // Extract features in the image
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      fast_feature_detector_->detectAndCompute(cv_ptr->image, cv::noArray(), keypoints,
                                               descriptors);
      for (size_t i = 0; i < keypoints.size(); i++) {
        datastructure::OrbFeature orb_feature;
        orb_feature.keypoint = keypoints[i];
        descriptors.row(i).copyTo(orb_feature.descriptor);
      }
    }
  }  // namespace feature_extraction_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    vslam_components::feature_extraction_nodes::FastFeatureExtractionNode)