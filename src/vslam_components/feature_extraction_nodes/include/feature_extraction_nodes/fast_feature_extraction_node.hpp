// TODO: Licence

#ifndef FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_
#define FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
  namespace datastructure {
    struct OrbFeature {
      cv::KeyPoint keypoint;
      cv::Mat descriptor;
    };
  }  // namespace datastructure

  namespace feature_extraction_nodes {
    class FastFeatureExtractionNode : public rclcpp::Node {
    public:
      explicit FastFeatureExtractionNode(const rclcpp::NodeOptions &options);

    private:
      void frame_callback(const vslam_msgs::msg::Frame::SharedPtr frame_msg) const;

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      cv::Ptr<cv::ORB> fast_feature_detector_;
    };
  }  // namespace feature_extraction_nodes
}  // namespace vslam_components

#endif  // FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_