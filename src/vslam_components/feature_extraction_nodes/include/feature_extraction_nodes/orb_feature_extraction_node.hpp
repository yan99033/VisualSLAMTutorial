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
      static constexpr int desc_len = 32;

      cv::KeyPoint keypoint;
      uint8_t descriptor[desc_len];
    };
  }  // namespace datastructure

  namespace feature_extraction_nodes {
    class OrbFeatureExtractionNode : public rclcpp::Node {
    public:
      explicit OrbFeatureExtractionNode(const rclcpp::NodeOptions &options);

    private:
      void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) const;

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;

      cv::Ptr<cv::ORB> orb_feature_detector_;
    };
  }  // namespace feature_extraction_nodes
}  // namespace vslam_components

#endif  // FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_