// TODO: Licence

#ifndef VSLAM_NODES__UTILS_HPP_
#define VSLAM_NODES__UTILS_HPP_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_msgs/msg/vector3d.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    using PointPairs = std::vector<std::pair<cv::Point3d, cv::Point3d>>;
    namespace utils {
      double calculate_sim3_scale(const PointPairs& mappoint_pairs, const cv::Mat& T_2_1, const int ransac_iters = 100,
                                  size_t ransac_n = 10);

      int encoding2mat_type(const std::string& encoding);

    }  // namespace utils

    namespace visualization {
      void add_keypoints_to_image_frame_msg(vslam_msgs::msg::Frame& frame_msg);

      visualization_msgs::msg::Marker calculate_pose_marker(
          const geometry_msgs::msg::Pose& pose, const std::string& frame_id, const double scale,
          const double line_thickness, const std::string& marker_ns, const int marker_id,
          const std::array<double, 3>& rgb = {0, 0, 0},
          const Eigen::Matrix3d& cam_axes_transform = Eigen::Matrix3d::Identity(),
          const rclcpp::Duration& duration = rclcpp::Duration({10000000}));

      visualization_msgs::msg::Marker calculate_mappoints_marker(
          const std::vector<vslam_msgs::msg::Vector3d>& mps, const std::string& frame_id, const double scale,
          const std::string& marker_ns, const int marker_id, const std::array<double, 3>& rgb = {0, 0, 0},
          const Eigen::Matrix3d& cam_axes_transform = Eigen::Matrix3d::Identity(),
          const rclcpp::Duration& duration = rclcpp::Duration({10000000}));
    }  // namespace visualization
  }    // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__UTILS_HPP_