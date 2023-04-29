#include "vslam_nodes/indirect_vslam_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include "vslam_msgs/msg/vector2d.hpp"
#include "vslam_msgs/msg/vector3d.hpp"
#include "vslam_nodes/utils.h"

using std::placeholders::_1;

namespace {
  int encoding2mat_type(const std::string& encoding) {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    }
    throw std::runtime_error("Unsupported mat type");
  }

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

    IndirectVSlamNode::IndirectVSlamNode(const rclcpp::NodeOptions& options)
        : Node("vslam_node", options), K_{load_camera_info()} {
      // Feature extractor
      feature_extractor_ = feature_extractor_loader_.createSharedInstance(
          declare_parameter("feature_extractor_plugin_name", "UNDEFINED"));
      feature_extractor_->initialize(declare_parameter("num_features", 2000));

      // Feature matcher
      feature_matcher_
          = feature_matcher_loader_.createSharedInstance(declare_parameter("feature_matcher_plugin_name", "UNDEFINED"));
      feature_matcher_->initialize();

      // Camera tracker
      camera_tracker_
          = camera_tracker_loader_.createSharedInstance(declare_parameter("camera_tracker_plugin_name", "UNDEFINED"));
      camera_tracker_->initialize(K_);

      // Mapper
      mapper_ = mapper_loader_.createSharedInstance(declare_parameter("mapper_plugin_name", "UNDEFINED"));
      mapper_->initialize(K_);

      // Back-end
      backend_ = backend_loader_.createSharedInstance(declare_parameter("backend_plugin_name", "UNDEFINED"));
      backend_->initialize(K_, frame_visual_queue_);

      // Place recognition
      place_recognition_ = place_recognition_loader_.createSharedInstance(
          declare_parameter("place_recognition_plugin_name", "UNDEFINED"));
      place_recognition_->initialize(declare_parameter("place_recognition.input", ""), declare_parameter("top_k", 3),
                                     declare_parameter("place_recognition.score_thresh", 0.9),
                                     declare_parameter("place_recognition.ignore_last_n_keyframes", -1));

      // Frame subscriber and publishers
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>("in_frame", 10,
                                                               std::bind(&IndirectVSlamNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
      keyframe_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_keyframe", 10);

      frame_msg_queue_publisher_thread_ = std::thread(&IndirectVSlamNode::frame_visual_publisher_loop, this);
      place_recognition_thread_ = std::thread(&IndirectVSlamNode::place_recognition_loop, this);
    }

    IndirectVSlamNode::~IndirectVSlamNode() {
      frame_visual_queue_->stop();
      keyframe_id_queue_->stop();

      exit_thread_ = true;
      frame_msg_queue_publisher_thread_.join();
      place_recognition_thread_.join();
    }

    cv::Mat IndirectVSlamNode::load_camera_info() {
      double fx = declare_parameter("camera_params.fx", -1.0);
      double fy = declare_parameter("camera_params.fy", -1.0);
      double cx = declare_parameter("camera_params.cx", -1.0);
      double cy = declare_parameter("camera_params.cy", -1.0);

      if (fx == -1.0 || fy == -1.0 || cx == -1.0 || cy == -1.0) {
        throw std::runtime_error("Camera matrix not loaded!");
      }

      return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    void IndirectVSlamNode::frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width, encoding2mat_type(frame_msg->image.encoding),
                     frame_msg->image.data.data());

      // Extract features in the image
      auto points = feature_extractor_->extract_features(cv_mat);

      if (state_ == State::init) {
        auto frame = std::make_shared<vslam_datastructure::Frame>(K_);
        frame->from_msg(frame_msg.get());
        frame->set_points(points);
        frame->set_keyframe();

        backend_->add_keyframe(frame);

        state_ = State::attempt_init;
      } else if (state_ == State::attempt_init) {
        auto current_keyframe = backend_->get_current_keyframe();
        if (!current_keyframe->has_points() || points.empty()) {
          backend_->remove_keyframe(current_keyframe);
          state_ = State::init;
          return;
        }

        auto current_frame = std::make_shared<vslam_datastructure::Frame>(K_);
        current_frame->from_msg(frame_msg.get());
        current_frame->set_points(points);

        auto [matched_points, matched_index_pairs]
            = feature_matcher_->match_features(current_keyframe->get_points(), current_frame->get_points());

        const auto T_c_p = camera_tracker_->track_camera_2d2d(matched_points);
        // T_c_p_ = T_c_p;

        // Camera pose
        cv::Mat T_p_w = current_keyframe->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->set_pose(T_c_w);

        auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p);
        current_keyframe->set_map_points(new_mps, get_first_indices(matched_index_pairs), true);

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        current_frame->to_msg(frame_msg.get(), skip_loaded);

        state_ = State::tracking;
      } else if (state_ == State::tracking) {
        auto current_keyframe = backend_->get_current_keyframe();
        if (!current_keyframe->has_points() || points.empty()) {
          std::cout << "Current frame has no point to track" << std::endl;
          state_ = State::relocalization;
          return;
        }

        std::cout << "current keyframe has " << current_keyframe->get_num_mps() << " to track" << std::endl;

        auto current_frame = std::make_shared<vslam_datastructure::Frame>(K_);
        current_frame->from_msg(frame_msg.get());
        current_frame->set_points(points);

        auto [matched_points, matched_index_pairs]
            = feature_matcher_->match_features(current_keyframe->get_points(), current_frame->get_points());

        // Check if we have enough map points for camera tracking
        size_t num_matched_mps{0};
        if (!check_mps_quality(matched_points, min_num_mps_cam_tracking_, num_matched_mps)) {
          std::cout << "Insufficient amount of points (" << num_matched_mps << ") needed for tracking" << std::endl;
          state_ = State::relocalization;
          return;
        }

        const auto T_c_p = camera_tracker_->track_camera_3d2d(matched_points);  //, T_c_p_);
        // T_c_p_ = T_c_p;

        // Check the number of outliers in the calculating the camera pose
        size_t num_matched_inliers{0};
        if (!check_mps_quality(matched_points, min_num_cam_tracking_inliers_, num_matched_inliers)) {
          std::cout << "Insufficient amount of inlier points (" << num_matched_inliers << ") used for tracking"
                    << std::endl;
          state_ = State::relocalization;
          return;
        }

        // Camera pose
        cv::Mat T_p_w = current_keyframe->T_f_w();
        cv::Mat T_c_w = T_c_p * T_p_w;
        current_frame->set_pose(T_c_w);

        // Check if we need a new keyframe
        const cv::Mat R_c_p = T_c_p.rowRange(0, 3).colRange(0, 3);
        size_t num_kf_mps{0};
        if (!check_mps_quality(matched_points, min_num_kf_mps_, num_kf_mps)
            || rotation_matrix_exceed_threshold(R_c_p, max_rotation_rad_)) {
          // Set constraints between adjacent keyframes
          current_keyframe->add_T_this_other_kf(current_frame.get(), T_c_p.inv());
          current_frame->add_T_this_other_kf(current_keyframe.get(), T_c_p);
          current_frame->set_parent_keyframe(current_keyframe.get());

          // Set the current frame as keyframe
          current_frame->set_keyframe();
          const auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p);
          const auto [first_indices, second_indices] = get_first_and_second_indices(matched_index_pairs);
          current_keyframe->set_map_points(new_mps, first_indices, true);
          const auto new_old_mps = current_keyframe->get_map_points(first_indices);
          current_frame->set_map_points(new_old_mps, second_indices);
          backend_->add_keyframe(current_frame);

          // Add the frame to visual update queue
          vslam_msgs::msg::Frame kf_msg;
          current_keyframe->to_msg(&kf_msg);
          frame_visual_queue_->send(std::move(kf_msg));

          // Add the keyframe id to find a potential loop
          long unsigned int kf_id = current_frame->id();
          keyframe_id_queue_->send(std::move(kf_id));

          std::cout << "created a new keyframe " << std::endl;
        } else {
          std::cout << "Didn't create a new keyframe. Currently tracking " << num_kf_mps << " map points" << std::endl;
        }

        std::cout << "current keyframe has " << current_keyframe->get_num_mps() << " map points" << std::endl;

        // write pose to the frame message
        constexpr bool skip_loaded = true;
        constexpr bool no_points = true;
        current_frame->to_msg(frame_msg.get(), skip_loaded, no_points);
      } else {
        // State::relocalization
        std::cout << "Relocalization state. unimplemented!" << std::endl;
      }
      pub_ptr->publish(std::move(frame_msg));
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

    void IndirectVSlamNode::frame_visual_publisher_loop() {
      while (!exit_thread_) {
        auto keyframe_msg = frame_visual_queue_->receive();

        keyframe_pub_->publish(std::move(keyframe_msg));

        // Free CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      std::cout << "Terminated frame publisher loop" << std::endl;
    }

    void IndirectVSlamNode::place_recognition_loop() {
      while (!exit_thread_) {
        // Get a keyframe from the keyframe queue
        auto curr_kf_id = keyframe_id_queue_->receive();

        // Get the keyframe from the backend
        const auto current_keyframe = backend_->get_keyframe(curr_kf_id);

        if (current_keyframe == nullptr) {
          RCLCPP_DEBUG(this->get_logger(), "current_keyframe is a nullptr");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              continue;
            }

            cv::Mat T_p_c{cv::Mat::eye(4, 4, CV_64F)};
            double scale{0.0};
            if (verify_loop(current_keyframe, previous_keyframe, T_p_c, scale)) {
              std::cout << "found a potential loop between " << curr_kf_id << " and " << prev_kf_id << std::endl;
              std::cout << T_p_c << std::endl;
              std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            }
          }
        }

        // Free CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    bool IndirectVSlamNode::verify_loop(const vslam_datastructure::Frame* const current_keyframe,
                                        const vslam_datastructure::Frame* const previous_keyframe, cv::Mat& T_p_c,
                                        double& scale) {
      if (current_keyframe == nullptr || previous_keyframe == nullptr) {
        return false;
      }

      if (!current_keyframe->has_points() || !previous_keyframe->has_points()) {
        RCLCPP_ERROR(this->get_logger(), "there is no point to track in verifying a potential loop");
        return false;
      }

      auto [matched_points, matched_index_pairs]
          = feature_matcher_->match_features(current_keyframe->get_points(), previous_keyframe->get_points());

      // Check if we have enough map points for camera tracking
      size_t num_matched_mps{0};
      if (!check_mps_quality(matched_points, min_num_mps_cam_tracking_, num_matched_mps)) {
        RCLCPP_INFO(this->get_logger(), "Insufficient amount of points (%lu) needed for tracking", num_matched_mps);
        return false;
      }

      T_p_c = camera_tracker_->track_camera_3d2d(matched_points);

      // Check the number of outliers in the calculating the camera pose
      size_t num_matched_inliers{0};
      if (!check_mps_quality(matched_points, min_num_cam_tracking_inliers_, num_matched_inliers)) {
        RCLCPP_INFO(this->get_logger(), "Insufficient amount of inlier points (%lu) used for tracking",
                    num_matched_inliers);
        return false;
      }

      // Calculate the scale
      // 1. find the 3D point correspondences within the match
      // 2. Transform the points in the current and previous keyframes to their coordinate frame
      // 3. Calculate the scale of the transformation
      const auto mappoint_pairs = get_mappoint_correspondences(matched_points, current_keyframe, previous_keyframe);
      scale = utils::calculate_sim3_scale(mappoint_pairs, T_p_c);

      // DEBUG
      cv::Matx33d R = T_p_c.rowRange(0, 3).colRange(0, 3);
      cv::Mat t = T_p_c.rowRange(0, 3).colRange(3, 4);
      cv::Point3d t_pt = cv::Point3d(t);
      for (const auto& [mp1, mp2] : mappoint_pairs) {
        const auto mp2_prime = R * mp1 * scale + t_pt;
        std::cout << "scale: " << scale << std::endl;
        std::cout << "mp2: " << mp2.x << " " << mp2.y << " " << mp2.z << std::endl;
        std::cout << "mp2': " << mp2_prime.x << " " << mp2_prime.y << " " << mp2_prime.z << std::endl;
      }

      if (scale == 0) {
        return false;
      } else {
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