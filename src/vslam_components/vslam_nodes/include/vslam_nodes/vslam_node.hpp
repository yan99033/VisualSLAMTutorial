// TODO: Licence

#ifndef VSLAM_NODES__VSLAM_NODE_HPP_
#define VSLAM_NODES__VSLAM_NODE_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_feature_extraction_plugins/feature_extraction_base.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    class VSlamNode : public rclcpp::Node {
    public:
      explicit VSlamNode(const rclcpp::NodeOptions &options);

    private:
      void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) const;

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;

      // Feature extraction plugin
      std::shared_ptr<vslam_feature_extraction_base::FeatureExtraction> feature_extractor_;

      cv::Ptr<cv::ORB> orb_feature_detector_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_