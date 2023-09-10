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

#include "vslam_nodes/indirect_vslam_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include "vslam_datastructure/utils.hpp"
#include "vslam_msgs/msg/vector2d.hpp"
#include "vslam_msgs/msg/vector3d.hpp"
#include "vslam_nodes/utils.hpp"
#include "vslam_utils/converter.hpp"

using std::placeholders::_1;

namespace vslam_components {

  namespace vslam_nodes {
    IndirectVSlamNode::IndirectVSlamNode(const rclcpp::NodeOptions& options) : Node("vslam_node", options) {
      // Feature extractor
      feature_extractor_ = feature_extractor_loader_.createSharedInstance(
          declare_parameter("feature_extractor_plugin_name", "UNDEFINED"));
      feature_extractor_->initialize(declare_parameter("feature_extractor.num_features", 2000),
                                     vslam_datastructure::Point::Type::orb);

      // Feature matcher
      feature_matcher_
          = feature_matcher_loader_.createSharedInstance(declare_parameter("feature_matcher_plugin_name", "UNDEFINED"));
      feature_matcher_->initialize(vslam_datastructure::Point::Type::orb);

      // Camera tracker
      camera_tracker_
          = camera_tracker_loader_.createSharedInstance(declare_parameter("camera_tracker_plugin_name", "UNDEFINED"));
      camera_tracker_->initialize();

      // Mapper
      mapper_ = mapper_loader_.createSharedInstance(declare_parameter("mapper_plugin_name", "UNDEFINED"));
      mapper_->initialize();

      // Back-end
      backend_ = backend_loader_.createSharedInstance(declare_parameter("backend_plugin_name", "UNDEFINED"));
      backend_->initialize(&map_);

      // Place recognition
      enable_place_recognition_ = declare_parameter("place_recognition.enable", false);
      if (enable_place_recognition_) {
        place_recognition_ = place_recognition_loader_.createSharedInstance(
            declare_parameter("place_recognition_plugin_name", "UNDEFINED"));
        place_recognition_->initialize(declare_parameter("place_recognition.input", ""),
                                       declare_parameter("place_recognition.top_k", 3),
                                       declare_parameter("place_recognition.score_thresh", 0.9),
                                       declare_parameter("place_recognition.ignore_last_n_keyframes", -1));
        place_recognition_thread_ = std::thread(&IndirectVSlamNode::placeRecognitionLoop, this);
      }

      // Visualizer
      visualizer_ = visualizer_loader_.createSharedInstance(declare_parameter("visualizer_plugin_name", "UNDEFINED"));
      visualizer_->initialize();

      // Frame subscriber and publishers
      frame_subscriber_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&IndirectVSlamNode::frameCallback, this, _1));
    }

    IndirectVSlamNode::~IndirectVSlamNode() {
      // Unsubscribe from the frame topic
      frame_subscriber_.reset();

      keyframe_queue_->stop();

      exit_thread_ = true;
      if (place_recognition_thread_.joinable()) {
        place_recognition_thread_.join();
      }

      keyframe_queue_.reset();

      // Destroy the plugins
      feature_extractor_.reset();
      feature_matcher_.reset();
      camera_tracker_.reset();
      mapper_.reset();
      backend_.reset();
      place_recognition_.reset();
      visualizer_.reset();

      current_keyframe_.reset();
      loop_keyframe_.reset();

      map_.clear();

      std::cerr << "Terminated vslam node" << std::endl;
    }

    void IndirectVSlamNode::frameCallback(vslam_msgs::msg::Frame::SharedPtr frame_msg) {
      RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width,
                     vslam_utils::conversions::encodingToCvMatType(frame_msg->image.encoding),
                     frame_msg->image.data.data());

      // Extract features in the image
      auto points = feature_extractor_->extractFeatures(cv_mat);

      // Create a new frame
      auto current_frame = vslam_datastructure::Frame::createFromPoints(std::move(points));
      current_frame->fromMsg(frame_msg.get());

      processFrame(current_frame);

      // write pose to the frame message
      constexpr bool skip_loaded = true;
      constexpr bool no_mappoints = true;
      current_frame->toMsg(frame_msg.get(), skip_loaded, no_mappoints);

      // Publish frame markers
      visualizer_->addLiveFrame(*frame_msg);
    }

    bool IndirectVSlamNode::processFrameInit(vslam_datastructure::Frame::SharedPtr current_frame) {
      current_frame->setKeyframe();
      current_keyframe_ = current_frame;

      return true;
    }

    bool IndirectVSlamNode::processFrameAttemptInit(vslam_datastructure::Frame::SharedPtr current_frame) {
      if (!current_keyframe_->hasPoints() || !current_frame->hasPoints()) {
        current_keyframe_ = nullptr;
        return false;
      }

      RCLCPP_INFO(get_logger(), "Attempt to initialize a keyframe");

      auto matched_points = feature_matcher_->matchFeatures(current_keyframe_->points(), current_frame->points());

      // Get the tracked camera pose and check the tracking quality
      cv::Mat T_c_p;
      if (!camera_tracker_->trackCamera2d2d(matched_points, current_frame->K(), T_c_p)) {
        current_keyframe_ = nullptr;
        return false;
      }

      // Camera pose
      cv::Mat T_p_w = current_keyframe_->T_f_w();
      cv::Mat T_c_w = T_c_p * T_p_w;
      current_frame->setPose(T_c_w);

      auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());

      // Check if we have enough initial map points
      if (new_mps.size() < min_num_kf_mps_) {
        current_keyframe_ = nullptr;
        return false;
      }

      const auto [points1, _] = vslam_datastructure::utils::splitMatchedPoints(matched_points);
      current_keyframe_->setMappoints(new_mps, points1, true);
      map_.addKeyframe(current_keyframe_);
      current_keyframe_->active_tracking_state = true;

      return true;
    }

    bool IndirectVSlamNode::processFrameTracking(vslam_datastructure::Frame::SharedPtr current_frame) {
      if (!current_keyframe_->hasPoints() || !current_frame->hasPoints()) {
        RCLCPP_INFO(get_logger(), "Current frame has no point to track");
        current_keyframe_->active_tracking_state = false;
        return false;
      }

      cv::Mat T_c_p;
      vslam_datastructure::MatchedPoints matched_points;
      if (!trackCamera(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points)) {
        current_keyframe_->active_tracking_state = false;
        return false;
      }

      // Camera pose
      cv::Mat T_p_w = current_keyframe_->T_f_w();
      cv::Mat T_c_w = T_c_p * T_p_w;
      current_frame->setPose(T_c_w);

      // Check if we need a new keyframe
      const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
      const double rotation_angle = vslam_utils::conversions::rotationMatrixToRotationAngle(R_c_p);
      size_t num_kf_mps{0};
      if (!utils::numMpsInMatchPointsAboveThresh(matched_points, min_num_kf_mps_, num_kf_mps)
          || rotation_angle > max_rotation_rad_) {
        // Set the current frame as keyframe
        current_frame->setKeyframe();
        const auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());

        const auto [points1, points2] = vslam_datastructure::utils::splitMatchedPoints(matched_points);
        current_keyframe_->setMappoints(new_mps, points1);
        const auto new_old_mps = vslam_datastructure::utils::extractMappointsFromPoints(points1);
        current_frame->setMappoints(new_old_mps, points2, true);

        map_.addKeyframe(current_frame);
        current_keyframe_->active_tracking_state = false;
        current_keyframe_ = current_frame;
        current_keyframe_->active_tracking_state = true;

        // Run local BA
        backend_->runLocalBA();

        // Add the frame to visual update queue
        vslam_msgs::msg::Frame keyframe_msg;
        current_keyframe_->toMsg(&keyframe_msg);

        visualizer_->addKeyframe(keyframe_msg);

        if (enable_place_recognition_) {
          // Add the keyframe to the queue to find a potential loop
          keyframe_queue_->send(std::move(current_frame));
        }

        RCLCPP_INFO(this->get_logger(), "Created a new keyframe");
      }

      return true;
    }

    bool IndirectVSlamNode::processFrameRelocalization(vslam_datastructure::Frame::SharedPtr current_frame) {
      RCLCPP_INFO(this->get_logger(), "Relocalizating");
      cv::Mat T_c_p;
      vslam_datastructure::MatchedPoints matched_points;
      bool tracked{false};
      // Track current frame relative to the current keyframe
      if (trackCamera(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points)) {
        tracked = true;
        current_keyframe_->active_tracking_state = true;
      } else {
        if (loop_keyframe_ != nullptr) {
          // Track current frame relative to the keyframe found using place recognition
          std::lock_guard<std::mutex> lck(loop_keyframe_mutex_);
          if (trackCamera(loop_keyframe_.get(), current_frame.get(), T_c_p, matched_points)) {
            tracked = true;

            // Make the current keyframe the keyframe found using place recognition
            current_keyframe_ = loop_keyframe_;
            current_keyframe_->active_tracking_state = true;
            loop_keyframe_ = nullptr;
          }
        }

        // Check if we need a new keyframe
        if (tracked) {
          const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
          const double rotation_angle = vslam_utils::conversions::rotationMatrixToRotationAngle(R_c_p);
          size_t num_kf_mps{0};
          if (!utils::numMpsInMatchPointsAboveThresh(matched_points, min_num_kf_mps_, num_kf_mps)
              || rotation_angle > max_rotation_rad_) {
            // Set the current frame as keyframe
            current_frame->setKeyframe();
            const auto new_mps = mapper_->map(matched_points, current_keyframe_->T_f_w(), T_c_p, current_frame->K());

            const auto [points1, points2] = vslam_datastructure::utils::splitMatchedPoints(matched_points);
            current_keyframe_->setMappoints(new_mps, points1);
            const auto new_old_mps = vslam_datastructure::utils::extractMappointsFromPoints(points1);
            current_frame->setMappoints(new_old_mps, points2, true);

            map_.addKeyframe(current_frame);
            current_keyframe_->active_tracking_state = false;
            current_keyframe_ = current_frame;
            current_keyframe_->active_tracking_state = true;
          }
        }
      }

      return tracked;
    }

    bool IndirectVSlamNode::trackCamera(const vslam_datastructure::Frame* const frame1,
                                        const vslam_datastructure::Frame* const frame2, cv::Mat& T_2_1,
                                        vslam_datastructure::MatchedPoints& matched_points) {
      if (frame1 == nullptr || frame2 == nullptr) {
        return false;
      }

      matched_points = feature_matcher_->matchFeatures(frame1->points(), frame2->points());

      // Check if we have enough map points for camera tracking
      size_t num_matched_mps{0};
      if (!utils::numMpsInMatchPointsAboveThresh(matched_points, min_num_mps_cam_tracking_, num_matched_mps)) {
        RCLCPP_INFO(get_logger(), "Insufficient amount of points (%lu) needed for tracking", num_matched_mps);
        return false;
      }

      // Get the tracked camera pose and check the tracking quality
      if (!camera_tracker_->trackCamera3d2d(matched_points, frame2->K(), T_2_1)) {
        RCLCPP_INFO(get_logger(), "Bad tracking quality");
        return false;
      }

      // Check the number of outliers in the calculating the camera pose
      size_t num_matched_inliers{0};
      if (!utils::numMpsInMatchPointsAboveThresh(matched_points, min_num_cam_tracking_inliers_, num_matched_inliers)) {
        RCLCPP_INFO(get_logger(), "Insufficient amount of inlier points (%lu) used for tracking", num_matched_inliers);
        return false;
      }

      return true;
    }

    void IndirectVSlamNode::placeRecognitionLoop() {
      while (!exit_thread_) {
        // Get a keyframe from the keyframe queue
        auto current_keyframe = keyframe_queue_->receive();

        if (!current_keyframe || current_keyframe->isBad()) {
          continue;
        }

        const auto curr_kf_id = current_keyframe->id();

        // Query from database
        cv::Mat visual_features = vslam_datastructure::utils::extractDescriptorsFromPoints(current_keyframe->points());
        auto results = place_recognition_->query(visual_features);
        place_recognition_->addToDatabase(curr_kf_id, visual_features);

        // Verify any potential loops
        if (results.size() > 0) {
          for (const auto& [prev_kf_id, score] : results) {
            const auto previous_keyframe = map_.getKeyframe(prev_kf_id);

            if (!previous_keyframe || previous_keyframe->isBad()) {
              continue;
            }

            cv::Mat T_p_c{cv::Mat::eye(4, 4, CV_64F)};
            double scale{0.0};
            RCLCPP_DEBUG(this->get_logger(), "Trying to find a loop: %lu <-> %lu. (Score: %f)", curr_kf_id, prev_kf_id,
                         score);
            vslam_datastructure::PointMappointPairs point_mappoint_pairs;
            if (verifyLoop(current_keyframe.get(), previous_keyframe.get(), T_p_c, scale, point_mappoint_pairs)) {
              RCLCPP_INFO(this->get_logger(), "Found a loop: %lu <-> %lu. (Score: %f) (Scale: %f)", curr_kf_id,
                          prev_kf_id, score, scale);

              // Keep the keyframe so it can be used for relocalization
              {
                std::lock_guard<std::mutex> lck(loop_keyframe_mutex_);
                loop_keyframe_ = previous_keyframe;
              }

              bool refresh_visual{false};
              if (curr_kf_id - last_kf_loop_found_ > skip_n_after_loop_found_) {
                // Run pose-graph optimization
                backend_->addLoopConstraint(prev_kf_id, curr_kf_id, T_p_c, scale);
                last_kf_loop_found_ = curr_kf_id;

                // Fuse the matched new map points with the existing ones
                if (!previous_keyframe->active_ba_state && !previous_keyframe->active_tracking_state) {
                  previous_keyframe->fuseMappoints(point_mappoint_pairs);

                  refresh_visual = true;
                }

              } else {
                // The relative scale has to small to fuse the map points
                if ((scale < 1.1 && scale > 0.9)
                    && (!previous_keyframe->active_ba_state && !previous_keyframe->active_tracking_state)) {
                  // Fuse the matched new map points with the existing ones
                  previous_keyframe->fuseMappoints(point_mappoint_pairs);

                  refresh_visual = true;
                }
              }

              if (refresh_visual) {
                // Refresh the display
                const auto keyframe_msgs = map_.getAllKeyframeMsgs();

                visualizer_->replaceAllKeyframes(keyframe_msgs);
              }
            }
          }
        }
      }
    }

    bool IndirectVSlamNode::verifyLoop(const vslam_datastructure::Frame* const current_keyframe,
                                       const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_p_c,
                                       double& scale, vslam_datastructure::PointMappointPairs& point_mappoint_pairs) {
      if (current_keyframe == nullptr || previous_keyframe == nullptr) {
        return false;
      }

      if (!current_keyframe->hasPoints() || !previous_keyframe->hasPoints()) {
        RCLCPP_INFO(this->get_logger(), "there is no point to track in verifying a potential loop");
        return false;
      }

      vslam_datastructure::MatchedPoints matched_points;
      if (!trackCamera(current_keyframe, previous_keyframe, T_p_c, matched_points)) {
        return false;
      }

      if (cv::norm(T_p_c.rowRange(0, 3).colRange(3, 4)) > max_loop_translation_) {
        return false;
      }

      // Calculate the scale
      const auto mappoint_pairs = vslam_datastructure::utils::mappointCorrespondencesFromMatchedPoints(matched_points);

      if (mappoint_pairs.size() < min_num_mps_sim3_scale_) {
        RCLCPP_INFO(this->get_logger(),
                    "Insufficient amount of map point correspondences(%lu) for "
                    "calculating Sim(3) scale",
                    mappoint_pairs.size());
        return false;
      }

      scale = utils::calculateSim3Scale(mappoint_pairs, T_p_c);

      if (scale <= 0) {
        return false;
      } else {
        point_mappoint_pairs
            = vslam_datastructure::utils::secondPointFirstMappointPairsFromMatchedPoints(matched_points);
        return true;
      }
    }

  }  // namespace vslam_nodes
}  // namespace vslam_components

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vslam_components::vslam_nodes::IndirectVSlamNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::vslam_nodes::IndirectVSlamNode)