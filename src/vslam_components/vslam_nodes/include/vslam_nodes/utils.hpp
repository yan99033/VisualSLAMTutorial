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

#ifndef VSLAM_NODES__UTILS_HPP_
#define VSLAM_NODES__UTILS_HPP_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vslam_datastructure/point.hpp"
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

      /// Check if the number of number of map points in the matched points is above the threshold to determine the
      /// matching and tracking quality
      /**
       * \param[in] matched_points point correspondences
       * \param[in] goodness_threshold return true if the number of inlier map points is greater than this
       * \param[out] num_mps number of inlier map points found
       * \return a boolean indicating if the number of map points is above the threshold
       */
      bool numMpsInMatchPointsAboveThresh(const vslam_datastructure::MatchedPoints& matched_points,
                                          const size_t goodness_thresh, size_t& num_mps);

    }  // namespace utils
  }    // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__UTILS_HPP_