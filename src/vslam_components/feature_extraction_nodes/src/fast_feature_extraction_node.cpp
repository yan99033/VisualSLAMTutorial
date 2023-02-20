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
      captured_frame_pub_ = frame_pub_;
    }

    void FastFeatureExtractionNode::frame_callback(
        vslam_msgs::msg::Frame::UniquePtr frame_msg) const {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(),
                    "FastFeatureExtractionNode: unable to lock the publisher\n");
        return;
      }

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

      // Create points and descriptors
      frame_msg->points.resize(keypoints.size());
      for (size_t i = 0; i < keypoints.size(); i++) {
        datastructure::OrbFeature orb_feature;
        orb_feature.keypoint = keypoints[i];
        descriptors.row(i).copyTo(orb_feature.descriptor);
        auto orb_feature_p = reinterpret_cast<uint8_t *>(&orb_feature);

        frame_msg->points[i].feature_data.resize(sizeof(datastructure::OrbFeature));
        std::copy(frame_msg->points[i].feature_data.begin(),
                  frame_msg->points[i].feature_data.end(), orb_feature_p);

        frame_msg->points[i].has_mp = false;
      }

      pub_ptr->publish(std::move(frame_msg));
    }
  }  // namespace feature_extraction_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    vslam_components::feature_extraction_nodes::FastFeatureExtractionNode)