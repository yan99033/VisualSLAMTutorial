// TODO: Licence

#ifndef VSLAM_NODES__VSLAM_NODE_HPP_
#define VSLAM_NODES__VSLAM_NODE_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    class IndirectVSlamNode : public rclcpp::Node {
    public:
      explicit IndirectVSlamNode(const rclcpp::NodeOptions &options);

    private:
      void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) const;

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;

      // Feature extraction plugin
      pluginlib::ClassLoader<vslam_feature_extractor_base::FeatureExtractor>
          feature_extractor_loader_{"vslam_plugins_base",
                                    "vslam_feature_extractor_base::FeatureExtractor"};
      std::shared_ptr<vslam_feature_extractor_base::FeatureExtractor> feature_extractor_;

      // Feature matcher plugin
      pluginlib::ClassLoader<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_loader_{
          "vslam_plugins_base", "vslam_feature_matcher_base::FeatureMatcher"};
      std::shared_ptr<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_