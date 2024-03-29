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

#include "vslam_visualizer_plugins/rviz.hpp"

#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "vslam_utils/converter.hpp"

namespace {
  /// Add keypoints to the image in the frame message
  /**
   * \param[in] frame_msg Frame message
   */
  void addKeypointsToImageFrameMsg(vslam_msgs::msg::Frame& frame_msg) {
    // Create a cv::Mat from the image message (without copying).
    cv::Mat cv_mat(frame_msg.image.height, frame_msg.image.width,
                   vslam_utils::conversions::encodingToCvMatType(frame_msg.image.encoding),
                   frame_msg.image.data.data());

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

  /// Create pose marker from camera vertices
  /**
   * \param[in] cam_vertices camera vertices that forms the camera markers
   * \param[in] frame_id frame id
   * \param[in] line_thickness line thickness of the camera vertices
   * \param[in] marker_ns marker namespace
   * \param[in] rgb rgb values of the marker
   * \param[in] duration lifetime of the marker
   * \param[in] cam_axes_transform 3x3 transformation matrix applied to the marker vertices
   */
  visualization_msgs::msg::Marker toPoseMarker(const Mat3x16& cam_vertices, const std::string& frame_id,
                                               const double line_thickness, const std::string& marker_ns,
                                               const int marker_id, const std::array<double, 3>& rgb,
                                               const rclcpp::Duration& duration,
                                               const Eigen::Matrix3d& cam_axes_transform
                                               = Eigen::Matrix3d::Identity()) {
    visualization_msgs::msg::Marker pose_marker;
    // Add vertices to the marker msg
    pose_marker.header.frame_id = frame_id;
    pose_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
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

  /// Create map point marker from map points
  /**
   * \param[in] mps a vector of map points
   * \param[in] frame_id frame id
   * \param[in] marker_scale scale of the point
   * \param[in] marker_ns marker namespace
   * \param[in] rgb rgb values of the marker
   * \param[in] duration lifetime of the marker
   * \param[in] cam_axes_transform 3x3 transformation matrix applied to the marker vertices
   */
  visualization_msgs::msg::Marker toMappointsMarker(const std::vector<vslam_msgs::msg::Vector3d>& mps,
                                                    const std::string& frame_id, const double marker_scale,
                                                    const std::string& marker_ns, const int marker_id,
                                                    const std::array<double, 3>& rgb, const rclcpp::Duration& duration,
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

  /// Convert the camera pose to transform message
  /**
   * \param[in] T_w_c camera pose
   * \param[in] cam_axes_transform 3x3 transformation matrix applied to the marker vertices
   * \param[in] map_frame_id map frame id
   * \param[in] map_frame_id camera frame id
   * \param[in] stamp timestamp of the camera
   */
  geometry_msgs::msg::TransformStamped toTransformStamped(const Eigen::Isometry3d& T_w_c,
                                                          const Eigen::Matrix3d& cam_axes_transform,
                                                          const std::string& map_frame_id,
                                                          const std::string& cam_frame_id,
                                                          const builtin_interfaces::msg::Time& stamp) {
    Eigen::Vector3d translation = cam_axes_transform * T_w_c.translation();
    Eigen::Matrix3d rotation_matrix = cam_axes_transform * T_w_c.rotation() * cam_axes_transform.inverse();
    Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(rotation_matrix);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = map_frame_id;
    transform_stamped.child_frame_id = cam_frame_id;
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();

    return transform_stamped;
  }

  /// Convert pose message to 3D Eigen transformation matrix
  /**
   * \param[in] T_w_f pose message
   * \return Eigen transformation matrix
   */
  Eigen::Isometry3d toEigenIsometry3d(const geometry_msgs::msg::Pose& T_w_f) {
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  }

  void RViz::addLiveFrame(const vslam_msgs::msg::Frame& frame_msg) {
    auto frame_msg_cpy = frame_msg;
    addKeypointsToImageFrameMsg(frame_msg_cpy);
    image_publisher_->publish(frame_msg_cpy.image);

    Eigen::Isometry3d T_w_c = toEigenIsometry3d(frame_msg.pose);
    Mat3x16 cam_vertices = getCameraVertices(live_cam_marker_scale_, T_w_c);
    const auto pose_marker = toPoseMarker(cam_vertices, map_frame_id_, line_thickness_, live_cam_marker_ns_, -1,
                                          live_cam_marker_rgb_, rclcpp::Duration({10000000}), cam_axes_transform_);
    live_frame_publisher_->publish(pose_marker);

    geometry_msgs::msg::TransformStamped transform_stamped
        = toTransformStamped(T_w_c, cam_axes_transform_, map_frame_id_, cam_frame_id_, frame_msg.header.stamp);

    // Send the transformation
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  void RViz::addKeyframe(const vslam_msgs::msg::Frame& frame_msg) {
    Eigen::Isometry3d T_w_c = toEigenIsometry3d(frame_msg.pose);
    Mat3x16 cam_vertices = getCameraVertices(marker_scale_, T_w_c);
    const auto pose_marker
        = toPoseMarker(cam_vertices, map_frame_id_, line_thickness_, keyframe_cam_marker_ns_, frame_msg.id,
                       keyframe_cam_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);
    keyframe_publisher_->publish(pose_marker);

    const auto mps_marker
        = toMappointsMarker(frame_msg.mappoints, map_frame_id_, marker_scale_, keyframe_mps_marker_ns_, frame_msg.id,
                            mappoint_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);
    mappoint_publisher_->publish(mps_marker);
  }

  void RViz::removeKeyframe(const vslam_msgs::msg::Frame& frame_msg) {
    (void)frame_msg;

    RCLCPP_WARN(node_->get_logger(), "RViz::removeKeyframe: not supported! Use `RViz::replaceAllKeyframes` instead");
  }

  void RViz::replaceAllKeyframes(const FrameVec& frame_msgs) {
    if (frame_msgs.empty()) {
      return;
    }

    // Remove the old markers
    visualization_msgs::msg::Marker del_marker;
    del_marker.header.frame_id = map_frame_id_;
    del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    keyframe_publisher_->publish(del_marker);
    mappoint_publisher_->publish(del_marker);

    // Combine the new markers and publish them
    visualization_msgs::msg::Marker combined_pose_markers;
    visualization_msgs::msg::Marker combined_mps_markers;
    for (size_t i = 0; i < frame_msgs.size(); i++) {
      const auto frame_msg = frame_msgs[i];

      Eigen::Isometry3d T_w_c = toEigenIsometry3d(frame_msg.pose);
      Mat3x16 cam_vertices = getCameraVertices(marker_scale_, T_w_c);
      const auto pose_marker
          = toPoseMarker(cam_vertices, map_frame_id_, line_thickness_, keyframe_cam_marker_ns_, frame_msg.id,
                         keyframe_cam_marker_rgb_, rclcpp::Duration({0}), cam_axes_transform_);

      const auto mps_marker
          = toMappointsMarker(frame_msg.mappoints, map_frame_id_, marker_scale_, keyframe_mps_marker_ns_, frame_msg.id,
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