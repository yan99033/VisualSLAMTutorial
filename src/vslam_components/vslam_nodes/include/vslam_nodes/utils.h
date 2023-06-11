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
      /// Calculate the Sim(3) scale of the relative pose constraint
      /**
       * \param[in] mappoint_pairs map point correspondences
       * \param[in] T_2_1 relative transformation between the map points
       * \param[in] ransac_iters number of RANSAC iterations for calculating the Sim(3) scale
       * \param[in] ransac_n number of random samples used in each RANSAC iteration to calculate the Sim(3) scale
       * \return Sim(3) scale (return 0 if we cannot find the scale)
       */
      double calculateSim3Scale(const PointPairs& mappoint_pairs, const cv::Mat& T_2_1, const int ransac_iters = 100,
                                size_t ransac_n = 10);

    }  // namespace utils
  }    // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__UTILS_HPP_