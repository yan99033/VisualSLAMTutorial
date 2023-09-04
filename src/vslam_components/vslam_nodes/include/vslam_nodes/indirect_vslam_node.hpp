/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
#include "vslam_nodes/vslam_node_base.hpp"
#include "vslam_plugins_base/backend.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"
#include "vslam_plugins_base/mapper.hpp"
#include "vslam_plugins_base/place_recognition.hpp"
#include "vslam_plugins_base/visualizer.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    namespace abstract {
      class IndirectVSlamNode : public virtual abstract::VSlamNode {
      private:
        /// Callback function used by the frame subscriber to get a new frame
        /**
         * \param frame_msg[in] frame message (defined in vslam_interfaces)
         */
        virtual void frameCallback(vslam_msgs::msg::Frame::SharedPtr frame_msg) = 0;
      };
    }  // namespace abstract

    class IndirectVSlamNode : public rclcpp::Node, public virtual abstract::IndirectVSlamNode, public VSlamNode {
    public:
      explicit IndirectVSlamNode(const rclcpp::NodeOptions& options);

      ~IndirectVSlamNode();

    private:
      /// Callback function used by the frame subscriber to get a new frame
      /**
       * \param frame_msg[in] frame message (defined in vslam_interfaces)
       */
      void frameCallback(vslam_msgs::msg::Frame::SharedPtr frame_msg) override;

      /// Process the frame in the 'init' state
      /**
       * Set the current frame as the tentative current keyframe
       *
       * \param current_frame[in,out] current frame received from the camera
       * \return a boolean indicating if initialization is successful
       */
      bool processFrameInit(vslam_datastructure::Frame::SharedPtr current_frame) override;

      /// Process the frame in the 'attempt_init' state
      /**
       * Attempt to match feature and establish camera tracking between the current frame and the current keyframe
       * If tracking is successful, we create an initial set of map points for the current keyframe;
       *
       * \param current_frame[in,out] current frame received from the camera
       * \return a boolean indicating if the attempt to initialize is successful
       */
      bool processFrameAttemptInit(vslam_datastructure::Frame::SharedPtr current_frame) override;

      /// Process the frame in the 'tracking' state
      /**
       * Match features and track the camera pose between the current frame and the current keyframe
       *
       * \param current_frame[in,out] current frame received from the camera
       * \return a boolean indicating if tracking is successful
       */
      bool processFrameTracking(vslam_datastructure::Frame::SharedPtr current_frame) override;

      /// Process the frame in the 'relocalization' state
      /**
       * Relocalize frame
       *
       * \param current_frame[in,out] current frame received from the camera
       * \return a boolean indicating if relocalization is successful
       */
      bool processFrameRelocalization(vslam_datastructure::Frame::SharedPtr current_frame) override;

      /// Camera frame tracker to calculate the relative transform and the matched points
      /**
       * \param frame1[in] frame1 (usually the current keyframe)
       * \param frame2[in] frame2 (usually the current frame)
       * \param matched_points[out] Point correspondences
       * \param matched_index_pairs[out] Index of the matched points in frame1 and frame2
       * \return a boolean indicating the goodness of the tracking
       */
      bool trackCamera(const vslam_datastructure::Frame* const frame1, const vslam_datastructure::Frame* const frame2,
                       cv::Mat& T_2_1, vslam_datastructure::MatchedPoints& matched_points,
                       vslam_datastructure::MatchedIndexPairs& matched_index_pairs);

      /// Frame subscriber to get new frames from dataloader
      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_subscriber_;

      /// Current keyframe (used for camera tracking)
      vslam_datastructure::Frame::SharedPtr current_keyframe_{nullptr};

      /// The keyframe that is visually similar to the current frame determined by the place recognition algorithm
      vslam_datastructure::Frame::SharedPtr loop_keyframe_{nullptr};

      /// Mutex for getting and writing loop_keyframe_
      std::mutex loop_keyframe_mutex_;

      /// Minimum number of map points needed for camera tracking
      size_t min_num_mps_cam_tracking_{15};

      /// Minimum number of inliers needed for good camera tracking
      size_t min_num_cam_tracking_inliers_{10};

      /// Minimum number of map points needed for a keyframe
      size_t min_num_kf_mps_{180};

      /// Relative rotation threshond between the keyframe and the current frame. Make the current frame a keyframe if
      /// the relative rotation exceeds this threshold
      double max_rotation_rad_{0.174533};

      /// Minimum number of map point correspondences required for calculating Sim(3) scale
      size_t min_num_mps_sim3_scale_{25};

      /// Signal queue for keyframes to find potential loop
      vslam_datastructure::FrameIdQueue::SharedPtr keyframe_id_queue_{
          std::make_shared<vslam_datastructure::FrameIdQueue>()};

      /// Place recognition thread for detecting loop closure
      std::thread place_recognition_thread_;

      /// Place recognition loop running in parallel along the main thread to detect loop closure
      void placeRecognitionLoop();

      /// Verify the loop between the current keyframe and the loop keyframe candidate.
      /**
       * Steps involved in the verification:
       * - Track the relative transformation
       * - Get the map point correspondences (if tracking is good)
       * - Calculate the Sim(3) scale between the corresponding map points (if the number of correspondences is above
       * a threshold)
       * - return true if the Sim(3) scale is good
       * \param[in] current_keyframe current keyframe
       * \param[in] previous_keyframe previous keyframe (found by place recognition)
       * \param[out] T_c_p relative transformation between the current and previous keyframe
       * \param[out] scale Sim(3) scale
       * \param[out] mappoint_index_pairs the corresponding indices of the matched points
       */
      bool verifyLoop(const vslam_datastructure::Frame* const current_keyframe,
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

      bool enable_place_recognition_{false};

      /// Feature extraction plugin loader
      pluginlib::ClassLoader<vslam_feature_extractor::base::FeatureExtractor> feature_extractor_loader_{
          "vslam_plugins_base", "vslam_feature_extractor::base::FeatureExtractor"};

      /// Feature extraction plugin
      std::shared_ptr<vslam_feature_extractor::base::FeatureExtractor> feature_extractor_;

      /// Feature matcher plugin loader
      pluginlib::ClassLoader<vslam_feature_matcher::base::FeatureMatcher> feature_matcher_loader_{
          "vslam_plugins_base", "vslam_feature_matcher::base::FeatureMatcher"};

      /// Feature matcher plugin
      std::shared_ptr<vslam_feature_matcher::base::FeatureMatcher> feature_matcher_;

      /// Camera tracker plugin loader
      pluginlib::ClassLoader<vslam_camera_tracker::base::CameraTracker> camera_tracker_loader_{
          "vslam_plugins_base", "vslam_camera_tracker::base::CameraTracker"};

      /// Camera tracker plugin
      std::shared_ptr<vslam_camera_tracker::base::CameraTracker> camera_tracker_;

      /// Mapper plugin laoder
      pluginlib::ClassLoader<vslam_mapper::base::Mapper> mapper_loader_{"vslam_plugins_base",
                                                                        "vslam_mapper::base::Mapper"};

      /// Mapper plugin
      std::shared_ptr<vslam_mapper::base::Mapper> mapper_;

      /// Back-end plugin loader
      pluginlib::ClassLoader<vslam_backend::base::Backend> backend_loader_{"vslam_plugins_base",
                                                                           "vslam_backend::base::Backend"};

      /// Back-end plugin
      std::shared_ptr<vslam_backend::base::Backend> backend_;

      /// Place recognition plugin loader
      pluginlib::ClassLoader<vslam_place_recognition::base::PlaceRecognition> place_recognition_loader_{
          "vslam_plugins_base", "vslam_place_recognition::base::PlaceRecognition"};

      /// Place recognition plugin
      std::shared_ptr<vslam_place_recognition::base::PlaceRecognition> place_recognition_;

      /// Visualizer plugin loader
      pluginlib::ClassLoader<vslam_visualizer::base::Visualizer> visualizer_loader_{
          "vslam_plugins_base", "vslam_visualizer::base::Visualizer"};

      /// Visualizer plugin
      std::shared_ptr<vslam_visualizer::base::Visualizer> visualizer_;
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_HPP_