// TODO: Licence

#ifndef VSLAM_NODES__VSLAM_NODE_HPP_
#define VSLAM_NODES__VSLAM_NODE_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"
#include "vslam_datastructure/signal_queue.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/backend.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"
#include "vslam_plugins_base/mapper.hpp"
#include "vslam_plugins_base/place_recognition.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    class IndirectVSlamNode : public rclcpp::Node {
    public:
      explicit IndirectVSlamNode(const rclcpp::NodeOptions& options);

      ~IndirectVSlamNode();

    private:
      enum class State : uint8_t { init = 0, attempt_init = 1, tracking = 2, relocalization = 3 };

      void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);

      // Check the goodness of the mapped points so far to determine the matching and tracking qualities
      bool check_mps_quality(const vslam_datastructure::MatchedPoints& matched_points, const size_t goodness_thresh,
                             size_t& num_mps);

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr keyframe_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;

      State state_{State::init};

      // vslam_datastructure::Frame::SharedPtr current_keyframe_;

      cv::Mat load_camera_info();

      cv::Mat K_;

      // Update the visualizer
      vslam_datastructure::FrameQueue::SharedPtr frame_visual_queue_{
          std::make_shared<vslam_datastructure::FrameQueue>()};

      // Thread to update publish the updated keyframes
      std::atomic_bool run_frame_visual_publisher_{true};
      std::thread frame_queue_publisher_thread_;
      void frame_visual_publisher_loop();

      // The previously calculated relative transformation for the initial guess in camera tracking
      cv::Mat T_c_p_{cv::Mat::eye(4, 4, CV_64F)};

      // Minimum number of map points needed for camera tracking
      size_t min_num_mps_cam_tracking_{30};

      // Minimum number of inliers needed for good camera tracking
      size_t min_num_cam_tracking_inliers_{15};

      // Minimum number of map points needed for a keyframe
      size_t min_num_kf_mps_{350};

      // Maximum allowable relative rotation between two keyframes. Defaults to 10 degree
      double max_rotation_rad_{0.174533};

      // loop-closure detection
      std::thread place_recognition_thread_;
      void place_recognition_loop();

      std::atomic_bool exit_thread_{false};

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

      // Back-end plugin
      pluginlib::ClassLoader<vslam_backend_base::Backend> backend_loader_{"vslam_plugins_base",
                                                                          "vslam_backend_base::Backend"};
      std::shared_ptr<vslam_backend_base::Backend> backend_;

      // Place recognition plugin
      pluginlib::ClassLoader<vslam_place_recognition_base::PlaceRecognition> place_recognition_loader_{
          "vslam_plugins_base", "vslam_place_recognition_base::PlaceRecognition"};
      std::shared_ptr<vslam_place_recognition_base::PlaceRecognition> place_recognition_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_