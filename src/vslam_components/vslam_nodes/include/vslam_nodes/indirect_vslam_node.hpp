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
      /// Camera tracking states
      /**
       * init: No keyframe is available for tracking. Tentatively set the current frame as a keyframe
       * attempt_init: Attempt to initialize the keyframe by calculating the relative transformation and map points
       *               between the keyframe and the current frame. Remove the keyframe and reset the state to init if
       *               initialization is not successful.
       * tracking: A keyframe is available for tracking. Track the current frame and determine if a new keyframe is
       *           needed. Triangulate new map points if a new keyframe is created
       * relocalization: Tracking is bad. Attempt to gain tracking using the current keyframe or the keyframe similar
       *                 to one of the keyframes found by place recognition
       */
      enum class State : uint8_t { init = 0, attempt_init = 1, tracking = 2, relocalization = 3 };

      /// Callback function used by the frame subscriber to get a new frame
      /**
       * \param frame_msg frame message (defined in vslam_interfaces)
       */
      void frame_callback(vslam_msgs::msg::Frame::SharedPtr frame_msg);

      /// Check the goodness of the mapped points so far to determine the matching and tracking qualities
      bool check_mps_quality(const vslam_datastructure::MatchedPoints& matched_points, const size_t goodness_thresh,
                             size_t& num_mps);

      /// Camera frame tracker to calculate the relative transform and the matched points
      /**
       * \param frame1 frame1 (usually the current keyframe)
       * \param frame2 frame2 (usually the current frame)
       * \param matched_points Point correspondences
       * \param matched_index_pairs Index of the matched points in frame1 and frame2
       * \return a boolean indicating the goodness of the tracking
       */
      bool track_camera(const vslam_datastructure::Frame* const frame1, const vslam_datastructure::Frame* const frame2,
                        cv::Mat& T_2_1, vslam_datastructure::MatchedPoints& matched_points,
                        vslam_datastructure::MatchedIndexPairs& matched_index_pairs);

      /// Frame subscriber to get new frames from dataloader
      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_subscriber_;

      /// State of the node
      State state_{State::init};

      /// Current keyframe (used for camera tracking)
      vslam_datastructure::Frame::SharedPtr current_keyframe_{nullptr};

      /// The keyframe that is visually similar to the current frame determined by the place recognition algorithm
      vslam_datastructure::Frame::SharedPtr loop_keyframe_{nullptr};

      /// Mutex for getting and writing loop_keyframe_
      std::mutex loop_keyframe_mutex_;

      /// Minimum number of map points needed for camera tracking
      size_t min_num_mps_cam_tracking_{20};

      /// Minimum number of inliers needed for good camera tracking
      size_t min_num_cam_tracking_inliers_{15};

      /// Minimum number of map points needed for a keyframe
      size_t min_num_kf_mps_{150};

      /// Relative rotation threshond between the keyframe and the current frame. Make the current frame a keyframe if
      /// the relative rotation exceeds this threshold
      double max_rotation_rad_{0.174533};

      /// Minimum number of map point correspondences required for calculating Sim(3) scale
      size_t min_num_mps_sim3_scale_{50};

      /// Signal queue for keyframes to find potential loop
      vslam_datastructure::FrameIdQueue::SharedPtr keyframe_id_queue_{
          std::make_shared<vslam_datastructure::FrameIdQueue>()};

      /// Place recognition thread for detecting loop closure
      std::thread place_recognition_thread_;

      /// Place recognition loop running in parallel along the main thread to detect loop closure
      void place_recognition_loop();

      /// Verify the loop between the current keyframe and the loop keyframe candidate.
      /// Steps in the verification:
      /// - Track the relative transformation
      /// - Get the map point correspondences (if tracking is good)
      /// - Calculate the Sim(3) scale between the corresponding map points (if the number of correspondences is above a
      ///     threshold)
      /// - return true if the Sim(3) scale is good
      bool verify_loop(const vslam_datastructure::Frame* const current_keyframe,
                       const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_c_p, double& scale,
                       std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>>& mappoint_index_pairs);

      /// The id of the keyframe successfully merged into the loop after place recognition and pose-graph optimization
      long unsigned int last_kf_loop_found_{0};

      /// The threshold of the relative translation when verifying a loop, which should be small regardless of scale
      double max_loop_translation_{0.5};

      /// Skip running pose-graph optimization after one has been found for n frames
      /// Instead, we fuse the map points if we can verify more loops between the skipped optimizations
      /// \note Having too many pose-graph optimizations in close vicinity would lag the optimization
      long unsigned int skip_n_after_loop_found_{50};

      /// Flag to exit the place recognition thread
      std::atomic_bool exit_thread_{false};

      /// Feature extraction plugin
      pluginlib::ClassLoader<vslam_feature_extractor::base::FeatureExtractor> feature_extractor_loader_{
          "vslam_plugins_base", "vslam_feature_extractor::base::FeatureExtractor"};
      std::shared_ptr<vslam_feature_extractor::base::FeatureExtractor> feature_extractor_;

      /// Feature matcher plugin
      pluginlib::ClassLoader<vslam_feature_matcher::base::FeatureMatcher> feature_matcher_loader_{
          "vslam_plugins_base", "vslam_feature_matcher::base::FeatureMatcher"};
      std::shared_ptr<vslam_feature_matcher::base::FeatureMatcher> feature_matcher_;

      /// Camera tracker plugin
      pluginlib::ClassLoader<vslam_camera_tracker::base::CameraTracker> camera_tracker_loader_{
          "vslam_plugins_base", "vslam_camera_tracker::base::CameraTracker"};
      std::shared_ptr<vslam_camera_tracker::base::CameraTracker> camera_tracker_;

      /// Mapper plugin
      pluginlib::ClassLoader<vslam_mapper::base::Mapper> mapper_loader_{"vslam_plugins_base",
                                                                        "vslam_mapper::base::Mapper"};
      std::shared_ptr<vslam_mapper::base::Mapper> mapper_;

      /// Back-end plugin
      pluginlib::ClassLoader<vslam_backend::base::Backend> backend_loader_{"vslam_plugins_base",
                                                                           "vslam_backend::base::Backend"};
      std::shared_ptr<vslam_backend::base::Backend> backend_;

      /// Place recognition plugin
      pluginlib::ClassLoader<vslam_place_recognition::base::PlaceRecognition> place_recognition_loader_{
          "vslam_plugins_base", "vslam_place_recognition::base::PlaceRecognition"};
      std::shared_ptr<vslam_place_recognition::base::PlaceRecognition> place_recognition_;

      /// Visualizer plugin
      pluginlib::ClassLoader<vslam_visualizer::base::Visualizer> visualizer_loader_{
          "vslam_plugins_base", "vslam_visualizer::base::Visualizer"};
      std::shared_ptr<vslam_visualizer::base::Visualizer> visualizer_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_