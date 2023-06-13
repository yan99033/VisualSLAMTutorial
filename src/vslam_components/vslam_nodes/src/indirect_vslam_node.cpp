/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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

#include "vslam_msgs/msg/vector2d.hpp"
#include "vslam_msgs/msg/vector3d.hpp"
#include "vslam_nodes/utils.h"
#include "vslam_utils/converter.hpp"

using std::placeholders::_1;

namespace {
  /// Get the first indices in the index pairs
  /**
   * \param[in] index_pairs a vector containing pairs of indices
   * \return a vector containing the first indices in the pairs
   */
  std::vector<size_t> getFirstIndices(const std::vector<std::pair<size_t, size_t>>& index_pairs) {
    std::vector<size_t> first_indices;
    for (const auto& [idx1, _] : index_pairs) {
      first_indices.push_back(idx1);
    }
    return first_indices;
  }

  /// Get the first and second indices in the index pairs
  /**
   * \param[in] index_pairs a vector containing pairs of indices
   * \return a pair of vector containing the first and second indices in the pairs
   */
  std::pair<std::vector<size_t>, std::vector<size_t>> getFirstAndSecondIndices(
      const std::vector<std::pair<size_t, size_t>>& index_pairs) {
    std::vector<size_t> first_indices;
    std::vector<size_t> second_indices;
    for (const auto& [idx1, idx2] : index_pairs) {
      first_indices.push_back(idx1);
      second_indices.push_back(idx2);
    }
    return {first_indices, second_indices};
  }

  /// Get the corresponding map points from the point correspondences
  /**
   * \param[in] matched_points point correspondences
   * \param[in] frame1 frame 1
   * \param[in] frame2 frame 2
   * \return a vector containing pairs of corresponding map points
   */
  std::vector<std::pair<cv::Point3d, cv::Point3d>> mappointCorrespondences(
      const vslam_datastructure::MatchedPoints& matched_points, const vslam_datastructure::Frame* const frame1,
      const vslam_datastructure::Frame* const frame2) {
    std::vector<std::pair<cv::Point3d, cv::Point3d>> mappoint_pairs;

    if (frame1 == nullptr || frame2 == nullptr) {
      return mappoint_pairs;
    }

    const cv::Mat T_1_w = frame1->T_f_w();
    cv::Matx33d R_1_w = T_1_w.rowRange(0, 3).colRange(0, 3);
    cv::Mat t_1_w = T_1_w.rowRange(0, 3).colRange(3, 4);
    cv::Point3d t_1_w_pt = cv::Point3d(t_1_w);

    const cv::Mat T_2_w = frame2->T_f_w();
    cv::Matx33d R_2_w = T_2_w.rowRange(0, 3).colRange(0, 3);
    cv::Mat t_2_w = T_2_w.rowRange(0, 3).colRange(3, 4);
    cv::Point3d t_2_w_pt = cv::Point3d(t_2_w);

    for (const auto& match : matched_points) {
      if (match.point1->hasMappoint() && match.point2->hasMappoint()) {
        cv::Point3d mp1 = match.point1->mappoint()->pos();
        mp1 = R_1_w * mp1 + t_1_w_pt;

        cv::Point3d mp2 = match.point2->mappoint()->pos();
        mp2 = R_2_w * mp2 + t_2_w_pt;
        mappoint_pairs.emplace_back(std::make_pair(mp1, mp2));
      }
    }

    return mappoint_pairs;
  }

  /// Get the corresponding map point indices and their map points to fuse (replacing the old map points with
  /// corresponding new ones)
  /**
   * map point indices are the indices to the old map points and they are replaced by the new map points
   * \param[in] matched_points point correspondences
   * \param[in] matched_index_pairs Index of the matched points in frame1 and frame2
   */
  std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> mappointIndexPairsToFuse(
      const vslam_datastructure::MatchedPoints& matched_points,
      const vslam_datastructure::MatchedIndexPairs& matched_index_pairs) {
    assert(matched_points.size() == matched_index_pairs.size());

    std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> mappoint_index_pairs;

    for (size_t i = 0; i < matched_points.size(); i++) {
      if (!matched_points.at(i).point1->hasMappoint()) {
        continue;
      }

      // Skip if the point has a map point but it is not the host of the map point
      if (matched_points.at(i).point2->hasMappoint() && !matched_points.at(i).point2->isMappointHost()) {
        continue;
      }

      mappoint_index_pairs.emplace_back(
          std::make_pair(matched_index_pairs.at(i).second, matched_points.at(i).point1->mappoint()));
    }

    return mappoint_index_pairs;
  }

  /// Check if the magnitude of the rotation exceeds the threshold
  /**
   * Calculate the magnitude of the rotation matrix by the angle of the axis-angle representation
   * \param[in] R rotation matrix
   * \param[in] angle_threshold_rad angle threshold
   * \return true if the rotation exceeds the threshold
   */
  bool rotationMatrixExceedThreshold(const cv::Mat& R, const double angle_threshold_rad) {
    assert(R.rows == 3 && R.cols == 3);

    // Calculate the angle component of the axis-angle representation
    const double tr = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    const double angle = acos((tr - 1) / 2);

    return angle > angle_threshold_rad;
  }

  /// Extract and concatenate descriptors from a vector of points
  /**
   * \param[in] points a vector of points
   * \return descriptors
   */
  cv::Mat extractDescriptors(const vslam_datastructure::Points& points) {
    std::vector<cv::Mat> descriptors_vec;
    for (const auto& pt : points) {
      descriptors_vec.push_back(pt->descriptor);
    }
    cv::Mat descriptors;
    cv::vconcat(descriptors_vec, descriptors);

    return descriptors;
  }
}  // namespace

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
      backend_->initialize();

      // Place recognition
      place_recognition_ = place_recognition_loader_.createSharedInstance(
          declare_parameter("place_recognition_plugin_name", "UNDEFINED"));
      place_recognition_->initialize(declare_parameter("place_recognition.input", ""),
                                     declare_parameter("place_recognition.top_k", 3),
                                     declare_parameter("place_recognition.score_thresh", 0.9),
                                     declare_parameter("place_recognition.ignore_last_n_keyframes", -1));

      // Visualizer
      visualizer_ = visualizer_loader_.createSharedInstance(declare_parameter("visualizer_plugin_name", "UNDEFINED"));
      visualizer_->initialize();

      // Frame subscriber and publishers
      frame_subscriber_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&IndirectVSlamNode::frameCallback, this, _1));

      place_recognition_thread_ = std::thread(&IndirectVSlamNode::placeRecognitionLoop, this);
    }

    IndirectVSlamNode::~IndirectVSlamNode() {
      // Unsubscribe from the frame topic
      frame_subscriber_.reset();

      keyframe_id_queue_->stop();

      exit_thread_ = true;
      place_recognition_thread_.join();

      feature_extractor_.reset();
      feature_matcher_.reset();
      camera_tracker_.reset();
      mapper_.reset();
      backend_.reset();
      place_recognition_.reset();
      visualizer_.reset();

      current_keyframe_.reset();
      loop_keyframe_.reset();

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
      auto current_frame = std::make_shared<vslam_datastructure::Frame>();
      current_frame->fromMsg(frame_msg.get());
      current_frame->setPoints(points);

      if (state_ == State::init) {
        current_frame->setKeyframe();

        backend_->addKeyfame(current_frame);
        current_keyframe_ = current_frame;

        state_ = State::attempt_init;
      } else if (state_ == State::attempt_init) {
        if (!current_keyframe_->hasPoints() || !current_frame->hasPoints()) {
          backend_->removeKeyframe(current_keyframe_);
          current_keyframe_ = nullptr;
          state_ = State::init;
          return;
        }

        auto [matched_points, matched_index_pairs]
            = feature_matcher_->matchFeatures(current_keyframe_->points(), current_frame->points());

        // Get the tracked camera pose and check the tracking quality
        cv::Mat T_c_p;
        if (!camera_tracker_->trackCamera2d2d(matched_points, current_frame->K(), T_c_p)) {
          backend_->removeKeyframe(current_keyframe_);
          current_keyframe_ = nullptr;
          state_ = State::init;
          return;
        }

        // Camera pose
        cv::Mat T_p_w = current_keyframe_->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->setPose(T_c_w);

        auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());

        // Check if we have enough initial map points
        if (new_mps.size() < min_num_kf_mps_) {
          backend_->removeKeyframe(current_keyframe_);
          current_keyframe_ = nullptr;
          state_ = State::init;
          return;
        }

        current_keyframe_->setMappoints(new_mps, getFirstIndices(matched_index_pairs), true);

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        current_frame->toMsg(frame_msg.get(), skip_loaded);

        current_keyframe_->active_tracking_state = true;

        state_ = State::tracking;
      } else if (state_ == State::tracking) {
        if (!current_keyframe_->hasPoints() || !current_frame->hasPoints()) {
          RCLCPP_INFO(get_logger(), "Current frame has no point to track");
          current_keyframe_->active_tracking_state = false;
          state_ = State::relocalization;
          return;
        }

        cv::Mat T_c_p;
        vslam_datastructure::MatchedPoints matched_points;
        vslam_datastructure::MatchedIndexPairs matched_index_pairs;
        if (!trackCamera(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
          current_keyframe_->active_tracking_state = false;
          state_ = State::relocalization;
          return;
        }

        // Camera pose
        cv::Mat T_p_w = current_keyframe_->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->setPose(T_c_w);

        // Check if we need a new keyframe
        const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
        size_t num_kf_mps{0};
        if (!checkMpsQuality(matched_points, min_num_kf_mps_, num_kf_mps)
            || rotationMatrixExceedThreshold(R_c_p, max_rotation_rad_)) {
          // Set constraints between adjacent keyframes
          current_keyframe_->addTThisOtherKf(current_frame.get(), T_c_p.inv());
          current_frame->addTThisOtherKf(current_keyframe_.get(), T_c_p);
          current_frame->setParentKeyframe(current_keyframe_.get());

          // Set the current frame as keyframe
          current_frame->setKeyframe();
          const auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());
          const auto [first_indices, second_indices] = getFirstAndSecondIndices(matched_index_pairs);
          current_keyframe_->setMappoints(new_mps, first_indices);
          const auto new_old_mps = current_keyframe_->mappoints(first_indices);
          current_frame->setMappoints(new_old_mps, second_indices, true);
          backend_->addKeyfame(current_frame);
          current_keyframe_->active_tracking_state = false;
          current_keyframe_ = current_frame;
          current_keyframe_->active_tracking_state = true;

          // Add the frame to visual update queue
          vslam_msgs::msg::Frame keyframe_msg;
          current_keyframe_->toMsg(&keyframe_msg);

          visualizer_->addKeyfame(keyframe_msg);

          // Add the keyframe id to find a potential loop
          long unsigned int kf_id = current_frame->id();
          keyframe_id_queue_->send(std::move(kf_id));

          RCLCPP_INFO(this->get_logger(), "Created a new keyframe");
        }

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        constexpr bool no_points = true;
        current_frame->toMsg(frame_msg.get(), skip_loaded, no_points);
      } else {
        RCLCPP_INFO(this->get_logger(), "Relocalizating");
        cv::Mat T_c_p;
        vslam_datastructure::MatchedPoints matched_points;
        vslam_datastructure::MatchedIndexPairs matched_index_pairs;
        bool tracked{false};
        // Track current frame relative to the current keyframe
        if (trackCamera(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
          tracked = true;
          current_keyframe_->active_tracking_state = true;
          state_ = State::tracking;
        } else {
          if (loop_keyframe_ != nullptr) {
            // Track current frame relative to the keyframe found using place recognition
            std::lock_guard<std::mutex> lck(loop_keyframe_mutex_);
            if (trackCamera(loop_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
              tracked = true;
              state_ = State::tracking;

              // Make the current keyframe the keyframe found using place recognition
              current_keyframe_ = loop_keyframe_;
              current_keyframe_->active_tracking_state = true;
              loop_keyframe_ = nullptr;
            }
          }

          // Check if we need a new keyframe
          if (tracked) {
            const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
            size_t num_kf_mps{0};
            if (!checkMpsQuality(matched_points, min_num_kf_mps_, num_kf_mps)
                || rotationMatrixExceedThreshold(R_c_p, max_rotation_rad_)) {
              // Set constraints between adjacent keyframes
              current_keyframe_->addTThisOtherKf(current_frame.get(), T_c_p.inv());
              current_frame->addTThisOtherKf(current_keyframe_.get(), T_c_p);
              current_frame->setParentKeyframe(current_keyframe_.get());

              // Set the current frame as keyframe
              current_frame->setKeyframe();
              const auto new_mps = mapper_->map(matched_points, current_keyframe_->T_f_w(), T_c_p, current_frame->K());
              const auto [first_indices, second_indices] = getFirstAndSecondIndices(matched_index_pairs);
              current_keyframe_->setMappoints(new_mps, first_indices);
              const auto new_old_mps = current_keyframe_->mappoints(first_indices);
              current_frame->setMappoints(new_old_mps, second_indices, true);
              backend_->addKeyfame(current_frame);
              current_keyframe_->active_tracking_state = false;
              current_keyframe_ = current_frame;
              current_keyframe_->active_tracking_state = true;
            }
          }
        }
      }

      // Publish frame markers
      visualizer_->addLiveFrame(*frame_msg);
    }

    bool IndirectVSlamNode::trackCamera(const vslam_datastructure::Frame* const frame1,
                                        const vslam_datastructure::Frame* const frame2, cv::Mat& T_2_1,
                                        vslam_datastructure::MatchedPoints& matched_points,
                                        vslam_datastructure::MatchedIndexPairs& matched_index_pairs) {
      if (frame1 == nullptr || frame2 == nullptr) {
        return false;
      }

      std::tie(matched_points, matched_index_pairs)
          = feature_matcher_->matchFeatures(frame1->points(), frame2->points());

      // Check if we have enough map points for camera tracking
      size_t num_matched_mps{0};
      if (!checkMpsQuality(matched_points, min_num_mps_cam_tracking_, num_matched_mps)) {
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
      if (!checkMpsQuality(matched_points, min_num_cam_tracking_inliers_, num_matched_inliers)) {
        RCLCPP_INFO(get_logger(), "Insufficient amount of inlier points (%lu) used for tracking", num_matched_inliers);
        return false;
      }

      return true;
    }

    bool IndirectVSlamNode::checkMpsQuality(const vslam_datastructure::MatchedPoints& matched_points,
                                            const size_t goodness_thresh, size_t& num_mps) {
      for (const auto& match : matched_points) {
        if (match.point1->hasMappoint()) {
          num_mps++;
        }

        if (num_mps > goodness_thresh) {
          return true;
        }
      }

      return false;
    }

    void IndirectVSlamNode::placeRecognitionLoop() {
      while (!exit_thread_) {
        // Get a keyframe from the keyframe queue
        auto curr_kf_id = keyframe_id_queue_->receive();

        // Get the keyframe from the backend
        const auto current_keyframe = backend_->getKeyframe(curr_kf_id);

        if (current_keyframe == nullptr) {
          RCLCPP_DEBUG(this->get_logger(), "current_keyframe is a nullptr");
          continue;
        }

        // Query from database
        cv::Mat visual_features = extractDescriptors(current_keyframe->points());
        auto results = place_recognition_->query(visual_features);
        place_recognition_->addToDatabase(curr_kf_id, visual_features);

        // Verify any potential loops
        if (results.size() > 0) {
          for (const auto& [prev_kf_id, score] : results) {
            const auto previous_keyframe = backend_->getKeyframe(prev_kf_id);

            if (previous_keyframe == nullptr) {
              RCLCPP_DEBUG(this->get_logger(), "current_keyframe is a nullptr");
              continue;
            }

            cv::Mat T_p_c{cv::Mat::eye(4, 4, CV_64F)};
            double scale{0.0};
            RCLCPP_DEBUG(this->get_logger(), "Trying to find a loop: %lu <-> %lu. (Score: %f)", curr_kf_id, prev_kf_id,
                         score);
            std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> mappoint_index_pairs;
            if (verifyLoop(current_keyframe.get(), previous_keyframe.get(), T_p_c, scale, mappoint_index_pairs)) {
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
                  previous_keyframe->fuseMappoints(mappoint_index_pairs);

                  refresh_visual = true;
                }

              } else {
                // The relative scale has to small to fuse the map points
                if ((scale < 1.1 && scale > 0.9)
                    && (!previous_keyframe->active_ba_state && !previous_keyframe->active_tracking_state)) {
                  // Fuse the matched new map points with the existing ones
                  previous_keyframe->fuseMappoints(mappoint_index_pairs);

                  refresh_visual = true;
                }
              }

              if (refresh_visual) {
                // Refresh the display
                const auto keyframe_msgs = backend_->getAllKeyframeMsgs();

                visualizer_->replaceAllKeyframes(keyframe_msgs);
              }
            }
          }
        }
      }
    }

    bool IndirectVSlamNode::verifyLoop(
        const vslam_datastructure::Frame* const current_keyframe,
        const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_p_c, double& scale,
        std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>>& mappoint_index_pairs) {
      if (current_keyframe == nullptr || previous_keyframe == nullptr) {
        return false;
      }

      if (!current_keyframe->hasPoints() || !previous_keyframe->hasPoints()) {
        RCLCPP_INFO(this->get_logger(), "there is no point to track in verifying a potential loop");
        return false;
      }

      vslam_datastructure::MatchedPoints matched_points;
      vslam_datastructure::MatchedIndexPairs matched_index_pairs;
      if (!trackCamera(current_keyframe, previous_keyframe, T_p_c, matched_points, matched_index_pairs)) {
        return false;
      }

      if (cv::norm(T_p_c.rowRange(0, 3).colRange(3, 4)) > max_loop_translation_) {
        return false;
      }

      // Calculate the scale
      const auto mappoint_pairs = mappointCorrespondences(matched_points, current_keyframe, previous_keyframe);

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
        mappoint_index_pairs = mappointIndexPairsToFuse(matched_points, matched_index_pairs);
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