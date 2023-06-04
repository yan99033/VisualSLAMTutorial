#include "vslam_visualizer_plugins/rviz.hpp"

#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "vslam_utils/converter.hpp"

namespace {

  void add_keypoints_to_image_frame_msg(vslam_msgs::msg::Frame& frame_msg) {
    // Create a cv::Mat from the image message (without copying).
    cv::Mat cv_mat(frame_msg.image.height, frame_msg.image.width,
                   vslam_utils::conversions::encoding2mat_type(frame_msg.image.encoding), frame_msg.image.data.data());

    // Add 2D keypoints to the image message and publish
    for (const auto& kp : frame_msg.keypoints) {
      constexpr int ft_radius = 5;
      const cv::Scalar ft_rgb(0, 0, 255);
      const int ft_thickness = -1;
      cv::circle(cv_mat, cv::Point2d{kp.x, kp.y}, ft_radius, ft_rgb, ft_thickness);

      constexpr int half_size = 5;
      const cv::Scalar sq_color(0, 255, 0);
      const int sq_thickness = 2;
      cv::rectangle(cv_mat, cv::Point2d{kp.x - half_size, kp.y - half_size},
                    cv::Point2d{kp.x + half_size, kp.y + half_size}, sq_color, sq_thickness);
    }
  }

  visualization_msgs::msg::Marker to_pose_marker(const Mat3x16& cam_vertices, const std::string& frame_id,
                                                 const double line_thickness, const std::string& marker_ns,
                                                 const int marker_id, const std::array<double, 3>& rgb,
                                                 const rclcpp::Duration& duration,
                                                 const Eigen::Matrix3d& cam_axes_transform
                                                 = Eigen::Matrix3d::Identity()) {
    visualization_msgs::msg::Marker pose_marker;
    // Add vertices to the marker msg
    pose_marker.header.frame_id = frame_id;
    pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.scale.x = line_thickness;
    pose_marker.color.r = rgb[0];
    pose_marker.color.g = rgb[1];
    pose_marker.color.b = rgb[2];
    pose_marker.color.a = 1.0;
    pose_marker.lifetime = duration;
    pose_marker.id = marker_id;
    pose_marker.ns = marker_ns;
    for (int i = 0; i < cam_vertices.cols(); i++) {
      Eigen::Vector3d cam_pt = cam_vertices.col(i);
      cam_pt = cam_axes_transform * cam_pt;
      auto pt = geometry_msgs::msg::Point();
      pt.x = cam_pt.x();
      pt.y = cam_pt.y();
      pt.z = cam_pt.z();

      pose_marker.points.push_back(pt);
    }

    return pose_marker;
  }

  visualization_msgs::msg::Marker to_mappoints_marker(const std::vector<vslam_msgs::msg::Vector3d>& mps,
                                                      const std::string& frame_id, const double marker_scale,
                                                      const std::string& marker_ns, const int marker_id,
                                                      const std::array<double, 3>& rgb,
                                                      const rclcpp::Duration& duration,
                                                      const Eigen::Matrix3d& cam_axes_transform) {
    visualization_msgs::msg::Marker mps_marker;
    mps_marker.header.frame_id = frame_id;
    mps_marker.type = visualization_msgs::msg::Marker::POINTS;
    mps_marker.action = visualization_msgs::msg::Marker::ADD;
    mps_marker.scale.x = marker_scale;
    mps_marker.scale.y = marker_scale;
    mps_marker.scale.z = marker_scale;
    mps_marker.color.r = rgb[0];
    mps_marker.color.g = rgb[1];
    mps_marker.color.b = rgb[2];
    mps_marker.color.a = 1.0;
    mps_marker.id = marker_id;
    mps_marker.ns = marker_ns;
    mps_marker.lifetime = duration;
    for (const auto& mp : mps) {
      Eigen::Vector3d mp_eigen(mp.x, mp.y, mp.z);
      mp_eigen = cam_axes_transform * mp_eigen;

      auto pt = geometry_msgs::msg::Point();
      pt.x = mp_eigen.x();
      pt.y = mp_eigen.y();
      pt.z = mp_eigen.z();
      mps_marker.points.push_back(pt);
    }

    return mps_marker;
  }

  Eigen::Isometry3d to_eigen_isometry3d(const geometry_msgs::msg::Pose& T_w_f) {
    // Convert the pose to an Eigen matrix
    Eigen::Isometry3d eigen_T_w_f;
    tf2::fromMsg(T_w_f, eigen_T_w_f);

    return eigen_T_w_f;
  }

}  // namespace

namespace vslam_visualizer_plugins {
  RViz::~RViz() { std::cerr << "Terminated RViz" << std::endl; }

  void RViz::initialize() {
    node_ = std::make_shared<rclcpp::Node>("visualizer_node", "vslam");

    image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("live_image", 1);
    live_frame_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("live_frame", 10);
    keyframe_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("keyframes", 10);
    mappoint_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("mappoints", 10);
  }

  void RViz::add_live_frame(const vslam_msgs::msg::Frame& frame_msg) {
    auto frame_msg_cpy = frame_msg;
    add_keypoints_to_image_frame_msg(frame_msg_cpy);
    image_publisher_->publish(frame_msg_cpy.image);

    Eigen::Isometry3d T_w_c = to_eigen_isometry3d(frame_msg.pose);
    Mat3x16 cam_vertices = get_camera_vertices(live_cam_marker_scale_, T_w_c);
    const auto pose_marker = to_pose_marker(cam_vertices, frame_id_, line_thickness_, live_cam_marker_ns_, -1,
                                            live_cam_marker_rgb_, rclcpp::Duration({10000000}), cam_axes_transform_);
    live_frame_publisher_->publish(pose_marker);
  }

  void RViz::add_keyframe(const vslam_msgs::msg::Frame& frame_msg) {
    Eigen::Isometry3d T_w_c = to_eigen_isometry3d(frame_msg.pose);
    Mat3x16 cam_vertices = get_camera_vertices(marker_scale_, T_w_c);
    const auto pose_marker
        = to_pose_marker(cam_vertices, frame_id_, line_thickness_, keyframe_cam_marker_ns_, frame_msg.id,
                         keyframe_cam_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);
    keyframe_publisher_->publish(pose_marker);

    const auto mps_marker
        = to_mappoints_marker(frame_msg.mappoints, frame_id_, marker_scale_, keyframe_mps_marker_ns_, frame_msg.id,
                              mappoint_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);
    mappoint_publisher_->publish(mps_marker);
  }

  void RViz::remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) {
    (void)frame_msg;

    std::cerr << "RViz::remove_keyframe: not supported! Use `RViz::replace_all_keyframes` instead" << std::endl;
  }

  void RViz::replace_all_keyframes(const FrameVec& frame_msgs) {
    if (frame_msgs.empty()) {
      return;
    }

    // Remove the old markers
    visualization_msgs::msg::Marker del_marker;
    del_marker.header.frame_id = frame_id_;
    del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    keyframe_publisher_->publish(del_marker);
    mappoint_publisher_->publish(del_marker);

    // Combine the new markers and publish them
    visualization_msgs::msg::Marker combined_pose_markers;
    visualization_msgs::msg::Marker combined_mps_markers;
    for (size_t i = 0; i < frame_msgs.size(); i++) {
      const auto frame_msg = frame_msgs[i];

      Eigen::Isometry3d T_w_c = to_eigen_isometry3d(frame_msg.pose);
      Mat3x16 cam_vertices = get_camera_vertices(marker_scale_, T_w_c);
      const auto pose_marker
          = to_pose_marker(cam_vertices, frame_id_, line_thickness_, keyframe_cam_marker_ns_, frame_msg.id,
                           keyframe_cam_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);

      const auto mps_marker
          = to_mappoints_marker(frame_msg.mappoints, frame_id_, marker_scale_, keyframe_mps_marker_ns_, frame_msg.id,
                                mappoint_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);

      if (i == 0) {
        combined_pose_markers = pose_marker;
        combined_mps_markers = mps_marker;
      } else {
        combined_pose_markers.points.insert(combined_pose_markers.points.end(), pose_marker.points.begin(),
                                            pose_marker.points.end());
        combined_mps_markers.points.insert(combined_mps_markers.points.end(), mps_marker.points.begin(),
                                           mps_marker.points.end());
      }
    }

    keyframe_publisher_->publish(combined_pose_markers);
    mappoint_publisher_->publish(combined_mps_markers);
  }

}  // namespace vslam_visualizer_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_visualizer_plugins::RViz, vslam_visualizer::base::Visualizer)