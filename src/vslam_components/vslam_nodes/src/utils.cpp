#include "vslam_nodes/utils.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <tf2_eigen/tf2_eigen.hpp>

namespace vslam_components {
  namespace vslam_nodes {
    namespace utils {
      double calculate_sim3_scale(const PointPairs& point_pairs, const cv::Mat& T_2_1, const int ransac_iters,
                                  size_t ransac_n) {
        // Rotation and translation
        cv::Matx33d R = T_2_1.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_2_1.rowRange(0, 3).colRange(3, 4);
        cv::Point3d t_pt = cv::Point3d(t);

        // Cap the data point to a max of half the size of point_pairs
        ransac_n = std::min(ransac_n, static_cast<size_t>(point_pairs.size() / 2));

        // random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, static_cast<int>(point_pairs.size() - 1));

        double smallest_err{std::numeric_limits<double>::max()};
        double best_scale{0.0};

        for (int i = 0; i < ransac_iters; i++) {
          // Randomly select n indices
          std::set<size_t> indices;
          while (indices.size() < ransac_n) {
            indices.insert(static_cast<size_t>(dis(gen)));
          }

          // calculate scale
          double numerator = 0;
          double denominator = 0;
          for (const auto& idx : indices) {
            const auto mp1 = point_pairs.at(idx).first;
            const auto mp2 = point_pairs.at(idx).second;

            const auto vec1 = R * mp1;
            const auto vec2 = mp2 - t_pt;

            numerator += vec1.dot(vec2);
            denominator += vec1.dot(vec1);
          }
          const double scale = [&] {
            if (denominator == 0.0) {
              return 0.0;
            } else {
              return numerator / denominator;
            }
          }();

          // calculate the fitting error
          double fitting_error{0.0};
          for (const auto& [mp1, mp2] : point_pairs) {
            fitting_error += cv::norm(R * mp1 + t_pt - mp2);
          }

          // if the error smaller than the smallest error
          if (fitting_error < smallest_err && scale != 0) {
            best_scale = scale;
            smallest_err = fitting_error;
          }
        }

        return best_scale;
      }

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
    }  // namespace utils

    namespace visualization {
      void add_keypoints_to_image_frame_msg(vslam_msgs::msg::Frame& frame_msg) {
        // Create a cv::Mat from the image message (without copying).
        cv::Mat cv_mat(frame_msg.image.height, frame_msg.image.width,
                       utils::encoding2mat_type(frame_msg.image.encoding), frame_msg.image.data.data());

        // Add 2D keypoints to the image message and publish
        for (size_t i = 0; i < frame_msg.keypoints.size(); i++) {
          auto kp = frame_msg.keypoints.at(i);
          auto has_mp = frame_msg.keypoints_has_mp.at(i);

          constexpr int ft_radius = 5;
          const cv::Scalar ft_rgb(0, 0, 255);
          const int ft_thickness = -1;
          cv::circle(cv_mat, cv::Point2d{kp.x, kp.y}, ft_radius, ft_rgb, ft_thickness);

          if (has_mp) {
            constexpr int half_size = 5;
            const cv::Scalar sq_color(0, 255, 0);
            const int sq_thickness = 2;
            cv::rectangle(cv_mat, cv::Point2d{kp.x - half_size, kp.y - half_size},
                          cv::Point2d{kp.x + half_size, kp.y + half_size}, sq_color, sq_thickness);
          }
        }
      }

      visualization_msgs::msg::Marker calculate_pose_marker(const geometry_msgs::msg::Pose& pose,
                                                            const std::string& frame_id, const double scale,
                                                            const double line_thickness, const std::string& marker_ns,
                                                            const int marker_id, const std::array<double, 3>& rgb,
                                                            const Eigen::Matrix3d& cam_axes_transform,
                                                            const rclcpp::Duration& duration) {
        // Convert the pose to an Eigen matrix
        Eigen::Isometry3d eigen_transform;
        tf2::fromMsg(pose, eigen_transform);

        // Calculate the preset vertices
        constexpr double fx = 340.0;
        constexpr double fy = 350.0;
        constexpr double cx = 300.0;
        constexpr double cy = 200.0;
        constexpr double w = 600.0;
        constexpr double h = 400.0;
        const double s = scale;
        const double p0[] = {0.0, 0.0, 0.0, 1.0};
        const double p1[] = {s * (0 - cx) / fx, s * (0 - cy) / fy, s, 1.0};
        const double p2[] = {s * (0 - cx) / fx, s * (h - 1 - cy) / fy, s, 1.0};
        const double p3[] = {s * (w - 1 - cx) / fx, s * (h - 1 - cy) / fy, s, 1.0};
        const double p4[] = {s * (w - 1 - cx) / fx, s * (0 - cy) / fy, s, 1.0};

        constexpr int num_vertices = 16;
        constexpr int num_dims = 4;
        Eigen::MatrixXd marker_vertices(num_dims, num_vertices);
        marker_vertices << p0[0], p1[0], p0[0], p2[0], p0[0], p3[0], p0[0], p4[0], p4[0], p3[0], p3[0], p2[0], p2[0],
            p1[0], p1[0], p4[0],  //
            p0[1], p1[1], p0[1], p2[1], p0[1], p3[1], p0[1], p4[1], p4[1], p3[1], p3[1], p2[1], p2[1], p1[1], p1[1],
            p4[1],                //
            p0[2], p1[2], p0[2], p2[2], p0[2], p3[2], p0[2], p4[2], p4[2], p3[2], p3[2], p2[2], p2[2], p1[2], p1[2],
            p4[2],                //
            p0[3], p1[3], p0[3], p2[3], p0[3], p3[3], p0[3], p4[3], p4[3], p3[3], p3[3], p2[3], p2[3], p1[3], p1[3],
            p4[3];                //
        marker_vertices = eigen_transform.matrix() * marker_vertices;
        marker_vertices.topRows(3) = cam_axes_transform * marker_vertices.topRows(3);

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
        for (int i = 0; i < num_vertices; i++) {
          auto pt = geometry_msgs::msg::Point();
          pt.x = marker_vertices.col(i)(0);
          pt.y = marker_vertices.col(i)(1);
          pt.z = marker_vertices.col(i)(2);

          pose_marker.points.push_back(pt);
        }

        return pose_marker;
      }

      visualization_msgs::msg::Marker calculate_mappoints_marker(const std::vector<vslam_msgs::msg::Vector3d>& mps,
                                                                 const std::string& frame_id, const double scale,
                                                                 const std::string& marker_ns, const int marker_id,
                                                                 const std::array<double, 3>& rgb,
                                                                 const Eigen::Matrix3d& cam_axes_transform,
                                                                 const rclcpp::Duration& duration) {
        visualization_msgs::msg::Marker mps_marker;
        mps_marker.header.frame_id = frame_id;
        mps_marker.type = visualization_msgs::msg::Marker::POINTS;
        mps_marker.action = visualization_msgs::msg::Marker::ADD;
        mps_marker.scale.x = scale;
        mps_marker.scale.y = scale;
        mps_marker.scale.z = scale;
        mps_marker.color.r = rgb[0];
        mps_marker.color.g = rgb[1];
        mps_marker.color.b = rgb[2];
        mps_marker.color.a = 1.0;
        mps_marker.id = marker_id;
        mps_marker.ns = marker_ns;
        for (const auto& mp : mps) {
          Eigen::Vector3d mp_eigen(mp.x, mp.y, mp.z);
          mp_eigen = cam_axes_transform * mp_eigen;

          auto pt = geometry_msgs::msg::Point();
          pt.x = mp_eigen.x();
          pt.y = mp_eigen.y();
          pt.z = mp_eigen.z();
          mps_marker.points.push_back(pt);
        }
      }

    }  // namespace visualization
  }    // namespace vslam_nodes
}  // namespace vslam_components