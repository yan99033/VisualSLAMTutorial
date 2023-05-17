#include "vslam_nodes/indirect_vslam_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include "vslam_msgs/msg/vector2d.hpp"
#include "vslam_msgs/msg/vector3d.hpp"
#include "vslam_nodes/utils.h"

using std::placeholders::_1;

namespace {
  std::vector<size_t> get_first_indices(const std::vector<std::pair<size_t, size_t>>& indice_pairs) {
    std::vector<size_t> first_indices;
    for (const auto& [idx1, _] : indice_pairs) {
      first_indices.push_back(idx1);
    }
    return first_indices;
  }

  std::pair<std::vector<size_t>, std::vector<size_t>> get_first_and_second_indices(
      const std::vector<std::pair<size_t, size_t>>& indice_pairs) {
    std::vector<size_t> first_indices;
    std::vector<size_t> second_indices;
    for (const auto& [idx1, idx2] : indice_pairs) {
      first_indices.push_back(idx1);
      second_indices.push_back(idx2);
    }
    return {first_indices, second_indices};
  }

  std::vector<std::pair<cv::Point3d, cv::Point3d>> get_mappoint_correspondences(
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
      if (match.point1->mappoint.get() && !match.point1->mappoint->is_outlier() && match.point2->mappoint.get()
          && !match.point2->mappoint->is_outlier()) {
        cv::Point3d mp1 = match.point1->mappoint->get_mappoint();
        mp1 = R_1_w * mp1 + t_1_w_pt;

        cv::Point3d mp2 = match.point2->mappoint->get_mappoint();
        mp2 = R_2_w * mp2 + t_2_w_pt;
        mappoint_pairs.emplace_back(std::make_pair(mp1, mp2));
      }
    }

    return mappoint_pairs;
  }

  std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> get_mappooint_index_pairs(
      const vslam_datastructure::MatchedPoints& matched_points,
      const vslam_datastructure::MatchedIndexPairs& matched_index_pairs) {
    assert(matched_points.size() == matched_index_pairs.size());

    std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> mappoint_index_pairs;

    for (size_t i = 0; i < matched_points.size(); i++) {
      if (matched_points.at(i).point2->mappoint.get() == nullptr
          || matched_points.at(i).point2->mappoint->is_outlier()) {
        continue;
      }

      mappoint_index_pairs.emplace_back(
          std::make_pair(matched_index_pairs.at(i).first, matched_points.at(i).point2->mappoint));
    }

    return mappoint_index_pairs;
  }

  bool rotation_matrix_exceed_threshold(const cv::Mat& R, const double angle_threshold_rad) {
    assert(R.rows == 3 && R.cols == 3);

    // Calculate the angle component of the axis-angle representation
    const double tr = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    const double angle = acos((tr - 1) / 2);

    return angle > angle_threshold_rad;
  }

  cv::Mat extract_descriptors(const vslam_datastructure::Points& points) {
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
      feature_extractor_
          = plugin_loader_.feature_extractor(declare_parameter("feature_extractor_plugin_name", "UNDEFINED"));
      feature_extractor_->initialize(declare_parameter("feature_extractor.num_features", 2000));

      // Feature matcher
      feature_matcher_ = plugin_loader_.feature_matcher(declare_parameter("feature_matcher_plugin_name", "UNDEFINED"));
      feature_matcher_->initialize();

      // Camera tracker
      camera_tracker_ = plugin_loader_.camera_tracker(declare_parameter("camera_tracker_plugin_name", "UNDEFINED"));
      camera_tracker_->initialize();

      // Mapper
      mapper_ = plugin_loader_.mapper(declare_parameter("mapper_plugin_name", "UNDEFINED"));
      mapper_->initialize();

      // Back-end
      backend_ = plugin_loader_.backend(declare_parameter("backend_plugin_name", "UNDEFINED"));
      backend_->initialize();

      // Place recognition
      place_recognition_
          = plugin_loader_.place_recognition(declare_parameter("place_recognition_plugin_name", "UNDEFINED"));
      place_recognition_->initialize(declare_parameter("place_recognition.input", ""),
                                     declare_parameter("place_recognition.top_k", 3),
                                     declare_parameter("place_recognition.score_thresh", 0.9),
                                     declare_parameter("place_recognition.ignore_last_n_keyframes", -1));

      // Visualizer
      visualizer_ = plugin_loader_.visualizer(declare_parameter("visualizer_plugin_name", "UNDEFINED"));
      visualizer_->initialize();

      // Frame subscriber and publishers
      frame_subscriber_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&IndirectVSlamNode::frame_callback, this, _1));

      place_recognition_thread_ = std::thread(&IndirectVSlamNode::place_recognition_loop, this);
    }

    IndirectVSlamNode::~IndirectVSlamNode() {
      keyframe_id_queue_->stop();

      exit_thread_ = true;
      place_recognition_thread_.join();
    }

    void IndirectVSlamNode::frame_callback(vslam_msgs::msg::Frame::SharedPtr frame_msg) {
      RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width,
                     utils::encoding2mat_type(frame_msg->image.encoding), frame_msg->image.data.data());

      // Extract features in the image
      auto points = feature_extractor_->extract_features(cv_mat);

      // Create a new frame
      auto current_frame = std::make_shared<vslam_datastructure::Frame>();
      current_frame->from_msg(frame_msg.get());
      current_frame->set_points(points);

      if (state_ == State::init) {
        current_frame->set_keyframe();

        backend_->add_keyframe(current_frame);
        current_keyframe_ = current_frame;

        state_ = State::attempt_init;
      } else if (state_ == State::attempt_init) {
        if (!current_keyframe_->has_points() || !current_frame->has_points()) {
          backend_->remove_keyframe(current_keyframe_);
          current_keyframe_ = nullptr;
          state_ = State::init;
          return;
        }

        auto [matched_points, matched_index_pairs]
            = feature_matcher_->match_features(current_keyframe_->get_points(), current_frame->get_points());

        const auto T_c_p = camera_tracker_->track_camera_2d2d(matched_points, current_frame->K());

        // Camera pose
        cv::Mat T_p_w = current_keyframe_->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->set_pose(T_c_w);

        auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());

        // Check if we have enough initial map points
        if (new_mps.size() < min_num_kf_mps_) {
          backend_->remove_keyframe(current_keyframe_);
          current_keyframe_ = nullptr;
          state_ = State::init;
          return;
        }

        current_keyframe_->set_mappoints(new_mps, get_first_indices(matched_index_pairs), true);

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        current_frame->to_msg(frame_msg.get(), skip_loaded);

        state_ = State::tracking;
      } else if (state_ == State::tracking) {
        if (!current_keyframe_->has_points() || !current_frame->has_points()) {
          RCLCPP_INFO(get_logger(), "Current frame has no point to track");
          state_ = State::relocalization;
          return;
        }

        cv::Mat T_c_p;
        vslam_datastructure::MatchedPoints matched_points;
        vslam_datastructure::MatchedIndexPairs matched_index_pairs;
        if (!camera_tracker(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
          state_ = State::relocalization;
          return;
        }

        // Camera pose
        cv::Mat T_p_w = current_keyframe_->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->set_pose(T_c_w);

        // Check if we need a new keyframe
        const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
        size_t num_kf_mps{0};
        if (!check_mps_quality(matched_points, min_num_kf_mps_, num_kf_mps)
            || rotation_matrix_exceed_threshold(R_c_p, max_rotation_rad_)) {
          // Set constraints between adjacent keyframes
          current_keyframe_->add_T_this_other_kf(current_frame.get(), T_c_p.inv());
          current_frame->add_T_this_other_kf(current_keyframe_.get(), T_c_p);
          current_frame->set_parent_keyframe(current_keyframe_.get());

          // Set the current frame as keyframe
          current_frame->set_keyframe();
          const auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p, current_frame->K());
          const auto [first_indices, second_indices] = get_first_and_second_indices(matched_index_pairs);
          current_keyframe_->set_mappoints(new_mps, first_indices);
          const auto new_old_mps = current_keyframe_->get_mappoints(first_indices);
          current_frame->set_mappoints(new_old_mps, second_indices, true);
          backend_->add_keyframe(current_frame);
          current_keyframe_ = current_frame;

          // Add the frame to visual update queue
          vslam_msgs::msg::Frame keyframe_msg;
          current_keyframe_->to_msg(&keyframe_msg);

          visualizer_->add_keyframe(keyframe_msg);

          // Add the keyframe id to find a potential loop
          long unsigned int kf_id = current_frame->id();
          keyframe_id_queue_->send(std::move(kf_id));

          RCLCPP_INFO(this->get_logger(), "Created a new keyframe");
        }

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        constexpr bool no_points = true;
        current_frame->to_msg(frame_msg.get(), skip_loaded, no_points);
      } else {
        RCLCPP_INFO(this->get_logger(), "Relocalizating");
        cv::Mat T_c_p;
        vslam_datastructure::MatchedPoints matched_points;
        vslam_datastructure::MatchedIndexPairs matched_index_pairs;
        bool tracked{false};
        // Track current frame relative to the current keyframe
        if (camera_tracker(current_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
          tracked = true;
          state_ = State::tracking;
        } else {
          if (loop_keyframe_ != nullptr) {
            // Track current frame relative to the keyframe found using place recognition
            std::lock_guard<std::mutex> lck(loop_keyframe_mutex_);
            if (camera_tracker(loop_keyframe_.get(), current_frame.get(), T_c_p, matched_points, matched_index_pairs)) {
              tracked = true;
              state_ = State::tracking;

              // Make the current keyframe the keyframe found using place recognition
              current_keyframe_ = loop_keyframe_;
              loop_keyframe_ = nullptr;
            }
          }

          // Check if we need a new keyframe
          if (tracked) {
            const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
            size_t num_kf_mps{0};
            if (!check_mps_quality(matched_points, min_num_kf_mps_, num_kf_mps)
                || rotation_matrix_exceed_threshold(R_c_p, max_rotation_rad_)) {
              // Set constraints between adjacent keyframes
              current_keyframe_->add_T_this_other_kf(current_frame.get(), T_c_p.inv());
              current_frame->add_T_this_other_kf(current_keyframe_.get(), T_c_p);
              current_frame->set_parent_keyframe(current_keyframe_.get());

              // Set the current frame as keyframe
              current_frame->set_keyframe();
              const auto new_mps = mapper_->map(matched_points, current_keyframe_->T_f_w(), T_c_p, current_frame->K());
              const auto [first_indices, second_indices] = get_first_and_second_indices(matched_index_pairs);
              current_keyframe_->set_mappoints(new_mps, first_indices);
              const auto new_old_mps = current_keyframe_->get_mappoints(first_indices);
              current_frame->set_mappoints(new_old_mps, second_indices, true);
              backend_->add_keyframe(current_frame);
              current_keyframe_ = current_frame;
            }
          }
        }
      }

      // Publish frame markers
      visualizer_->add_live_frame(*frame_msg);
    }

    bool IndirectVSlamNode::camera_tracker(const vslam_datastructure::Frame* const frame1,
                                           const vslam_datastructure::Frame* const frame2, cv::Mat& T_2_1,
                                           vslam_datastructure::MatchedPoints& matched_points,
                                           vslam_datastructure::MatchedIndexPairs& matched_index_pairs) {
      if (frame1 == nullptr || frame2 == nullptr) {
        return false;
      }

      std::tie(matched_points, matched_index_pairs)
          = feature_matcher_->match_features(frame1->get_points(), frame2->get_points());

      // Check if we have enough map points for camera tracking
      size_t num_matched_mps{0};
      if (!check_mps_quality(matched_points, min_num_mps_cam_tracking_, num_matched_mps)) {
        RCLCPP_INFO(get_logger(), "Insufficient amount of points (%lu) needed for tracking", num_matched_mps);
        return false;
      }

      T_2_1 = camera_tracker_->track_camera_3d2d(matched_points, frame2->K());

      // Check the number of outliers in the calculating the camera pose
      size_t num_matched_inliers{0};
      if (!check_mps_quality(matched_points, min_num_cam_tracking_inliers_, num_matched_inliers)) {
        RCLCPP_INFO(get_logger(), "Insufficient amount of inlier points (%lu) used for tracking", num_matched_inliers);
        return false;
      }

      return true;
    }

    bool IndirectVSlamNode::check_mps_quality(const vslam_datastructure::MatchedPoints& matched_points,
                                              const size_t goodness_thresh, size_t& num_mps) {
      for (const auto& match : matched_points) {
        if (match.point1->mappoint.get() && !match.point1->mappoint->is_outlier()) {
          num_mps++;
        }

        if (num_mps > goodness_thresh) {
          return true;
        }
      }

      return false;
    }

    void IndirectVSlamNode::place_recognition_loop() {
      while (!exit_thread_) {
        // Get a keyframe from the keyframe queue
        auto curr_kf_id = keyframe_id_queue_->receive();

        // Get the keyframe from the backend
        const auto current_keyframe = backend_->get_keyframe(curr_kf_id);

        if (curr_kf_id - last_kf_loop_found_ < skip_n_after_loop_found_) {
          cv::Mat visual_features = extract_descriptors(current_keyframe->get_points());
          place_recognition_->add_to_database(curr_kf_id, visual_features);

          continue;
        }

        if (current_keyframe == nullptr) {
          RCLCPP_DEBUG(this->get_logger(), "current_keyframe is a nullptr");
          continue;
        }

        // Query from database
        cv::Mat visual_features = extract_descriptors(current_keyframe->get_points());
        auto results = place_recognition_->query(visual_features);

        // Add visual features of the current frame to the database
        place_recognition_->add_to_database(curr_kf_id, visual_features);

        // Verify any potential loops
        if (results.size() > 0) {
          for (const auto& [prev_kf_id, score] : results) {
            const auto previous_keyframe = backend_->get_keyframe(prev_kf_id);

            if (previous_keyframe == nullptr) {
              RCLCPP_DEBUG(this->get_logger(), "current_keyframe is a nullptr");
              continue;
            }

            cv::Mat T_p_c{cv::Mat::eye(4, 4, CV_64F)};
            double scale{0.0};
            std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>> mappoint_index_pairs;
            if (verify_loop(current_keyframe.get(), previous_keyframe.get(), T_p_c, scale, mappoint_index_pairs)) {
              RCLCPP_INFO(this->get_logger(), "Found a loop: %lu <-> %lu. (Score: %f) (Scale: %f)", curr_kf_id,
                          prev_kf_id, score, scale);

              // Keep the keyframe so it can be used for relocalization
              {
                std::lock_guard<std::mutex> lck(loop_keyframe_mutex_);
                loop_keyframe_ = previous_keyframe;
              }

              // Run pose-graph optimization
              backend_->add_loop_constraint(prev_kf_id, curr_kf_id, T_p_c, scale);
              last_kf_loop_found_ = curr_kf_id;

              // Fuse the matched new map points with the existing ones
              current_keyframe->fuse_mappoints(mappoint_index_pairs);

              // Refresh the display
              const auto keyframe_msgs = backend_->get_all_keyframe_msgs();
              visualizer_->replace_all_keyframes(keyframe_msgs);
            }
          }
        }
      }
    }

    bool IndirectVSlamNode::verify_loop(
        const vslam_datastructure::Frame* const current_keyframe,
        const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_p_c, double& scale,
        std::vector<std::pair<size_t, vslam_datastructure::MapPoint::SharedPtr>>& mappoint_index_pairs) {
      if (current_keyframe == nullptr || previous_keyframe == nullptr) {
        return false;
      }

      if (!current_keyframe->has_points() || !previous_keyframe->has_points()) {
        RCLCPP_INFO(this->get_logger(), "there is no point to track in verifying a potential loop");
        return false;
      }

      vslam_datastructure::MatchedPoints matched_points;
      vslam_datastructure::MatchedIndexPairs matched_index_pairs;
      if (!camera_tracker(current_keyframe, previous_keyframe, T_p_c, matched_points, matched_index_pairs)) {
        return false;
      }

      if (cv::norm(T_p_c.rowRange(0, 3).colRange(3, 4)) > max_loop_translation_) {
        return false;
      }

      // Calculate the scale
      const auto mappoint_pairs = get_mappoint_correspondences(matched_points, current_keyframe, previous_keyframe);

      if (mappoint_pairs.size() < min_num_mps_sim3_scale_) {
        RCLCPP_INFO(this->get_logger(),
                    "Insufficient amount of map point correspondences(%lu) for calculating Sim(3) scale",
                    mappoint_pairs.size());
        return false;
      }

      scale = utils::calculate_sim3_scale(mappoint_pairs, T_p_c);

      if (scale <= 0) {
        return false;
      } else {
        mappoint_index_pairs = get_mappooint_index_pairs(matched_points, matched_index_pairs);
        return true;
      }
    }

  }  // namespace vslam_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::vslam_nodes::IndirectVSlamNode)