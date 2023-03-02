#include "feature_extraction_nodes/orb_feature_extraction_node.hpp"

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

  namespace feature_extraction_nodes {

    OrbFeatureExtractionNode::OrbFeatureExtractionNode(const rclcpp::NodeOptions &options)
        : Node("orb_feature_extraction_node", options) {
      // ORB feature detector
      orb_feature_detector_ = cv::ORB::create(declare_parameter("num_features", 2000));

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&OrbFeatureExtractionNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    void OrbFeatureExtractionNode::frame_callback(
        vslam_msgs::msg::Frame::UniquePtr frame_msg) const {
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
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      orb_feature_detector_->detectAndCompute(cv_mat, cv::noArray(), keypoints, descriptors);

      // Create points and descriptors
      frame_msg->points.resize(keypoints.size());
      for (size_t i = 0; i < keypoints.size(); i++) {
        datastructure::OrbFeature orb_feature;
        orb_feature.keypoint = keypoints[i];

        std::copy(descriptors.row(i).data, descriptors.row(i).data + orb_feature.desc_len,
                  orb_feature.descriptor);
        auto orb_feature_p = reinterpret_cast<uint8_t *>(&orb_feature);

        frame_msg->points[i].feature_data.resize(sizeof(datastructure::OrbFeature));
        std::memcpy(frame_msg->points[i].feature_data.data(), orb_feature_p,
                    sizeof(datastructure::OrbFeature));

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
    vslam_components::feature_extraction_nodes::OrbFeatureExtractionNode)