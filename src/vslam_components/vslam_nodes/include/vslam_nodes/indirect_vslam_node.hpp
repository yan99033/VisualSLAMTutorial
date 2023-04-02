// TODO: Licence

#ifndef VSLAM_NODES__VSLAM_NODE_HPP_
#define VSLAM_NODES__VSLAM_NODE_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"
#include "vslam_plugins_base/mapper.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    class IndirectVSlamNode : public rclcpp::Node {
    public:
      explicit IndirectVSlamNode(const rclcpp::NodeOptions &options);

    private:
      enum class State : uint8_t { init = 0, attempt_init = 1, tracking = 2, relocalization = 3 };

      void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;

      State state_{State::init};

      vslam_datastructure::Frame::SharedPtr current_keyframe_;

      cv::Mat load_camera_info();

      cv::Mat K_;

      // Feature extraction plugin
      pluginlib::ClassLoader<vslam_feature_extractor_base::FeatureExtractor> feature_extractor_loader_{
          "vslam_plugins_base", "vslam_feature_extractor_base::FeatureExtractor"};
      std::shared_ptr<vslam_feature_extractor_base::FeatureExtractor> feature_extractor_;

      // Feature matcher plugin
      pluginlib::ClassLoader<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_loader_{
          "vslam_plugins_base", "vslam_feature_matcher_base::FeatureMatcher"};
      std::shared_ptr<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_;

      // Camera tracker plugin
      pluginlib::ClassLoader<vslam_camera_tracker_base::CameraTracker> camera_tracker_loader_{
          "vslam_plugins_base", "vslam_camera_tracker_base::CameraTracker"};
      std::shared_ptr<vslam_camera_tracker_base::CameraTracker> camera_tracker_;

      // Mapper plugin
      pluginlib::ClassLoader<vslam_mapper_base::Mapper> mapper_loader_{"vslam_plugins_base",
                                                                       "vslam_mapper_base::Mapper"};
      std::shared_ptr<vslam_mapper_base::Mapper> mapper_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_