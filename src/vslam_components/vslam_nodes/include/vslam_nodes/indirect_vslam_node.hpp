// TODO: Licence

#ifndef VSLAM_NODES__VSLAM_NODE_HPP_
#define VSLAM_NODES__VSLAM_NODE_HPP_

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"
#include "vslam_datastructure/signal_queue.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_nodes/plugin_loader.hpp"
#include "vslam_plugins_base/backend.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"
#include "vslam_plugins_base/mapper.hpp"
#include "vslam_plugins_base/place_recognition.hpp"
#include "vslam_plugins_base/visualizer.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    class IndirectVSlamNode : public rclcpp::Node {
    public:
      explicit IndirectVSlamNode(const rclcpp::NodeOptions& options);

      ~IndirectVSlamNode();

    private:
      enum class State : uint8_t { init = 0, attempt_init = 1, tracking = 2, relocalization = 3 };

      void frame_callback(vslam_msgs::msg::Frame::SharedPtr frame_msg);

      // Check the goodness of the mapped points so far to determine the matching and tracking qualities
      bool check_mps_quality(const vslam_datastructure::MatchedPoints& matched_points, const size_t goodness_thresh,
                             size_t& num_mps);

      // Camera frame tracker
      bool camera_tracker(const vslam_datastructure::Frame* const frame1,
                          const vslam_datastructure::Frame* const frame2, cv::Mat& T_2_1,
                          vslam_datastructure::MatchedPoints& matched_points,
                          vslam_datastructure::MatchedIndexPairs& matched_index_pairs);

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_subscriber_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frame_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mappoint_publisher_;

      State state_{State::init};

      vslam_datastructure::Frame::SharedPtr current_keyframe_;

      // The keyframe that is visually similar to the current frame determined by the place recognition algorithm
      vslam_datastructure::Frame::SharedPtr loop_keyframe_{nullptr};
      std::mutex loop_keyframe_mutex_;

      // Minimum number of map points needed for camera tracking
      size_t min_num_mps_cam_tracking_{30};

      // Minimum number of inliers needed for good camera tracking
      size_t min_num_cam_tracking_inliers_{15};

      // Minimum number of map points needed for a keyframe
      size_t min_num_kf_mps_{250};

      // Maximum allowable relative rotation between two keyframes. Defaults to 10 degree
      double max_rotation_rad_{0.174533};

      // Minimum number of map point correspondences required for calculating Sim(3) scale
      size_t min_num_mps_sim3_scale_{100};

      // Signal queue for keyframes to find potential loop
      vslam_datastructure::FrameIdQueue::SharedPtr keyframe_id_queue_{
          std::make_shared<vslam_datastructure::FrameIdQueue>()};

      // loop-closure detection
      std::thread place_recognition_thread_;
      void place_recognition_loop();
      bool verify_loop(const vslam_datastructure::Frame* const current_keyframe,
                       const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_c_p, double& scale,
                       std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>>& mappoint_index_pairs);
      long unsigned int last_kf_loop_found_{0};

      // The magnitude of the relative translation between the two similar keyframes should be close to zero, regardless
      // of scale
      double max_loop_translation_{0.1};

      // Skip detecting loop after one has been found for n frames
      // Having too many loops in close vicinity would lag the optimization
      long unsigned int skip_n_after_loop_found_{50};

      // Flag to exit the frame visual publisher and place recognition threads
      std::atomic_bool exit_thread_{false};

      // Plugin loader
      vslam_plugins::Loader plugin_loader_;

      // Feature extraction plugin
      std::shared_ptr<vslam_feature_extractor_base::FeatureExtractor> feature_extractor_;

      // Feature matcher plugin
      std::shared_ptr<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_;

      // Camera tracker plugin
      std::shared_ptr<vslam_camera_tracker_base::CameraTracker> camera_tracker_;

      // Mapper plugin
      std::shared_ptr<vslam_mapper_base::Mapper> mapper_;

      // Back-end plugin
      std::shared_ptr<vslam_backend_base::Backend> backend_;

      // Place recognition plugin
      std::shared_ptr<vslam_place_recognition_base::PlaceRecognition> place_recognition_;

      // Visualizer plugin
      std::shared_ptr<vslam_visualizer_base::Visualizer> visualizer_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_