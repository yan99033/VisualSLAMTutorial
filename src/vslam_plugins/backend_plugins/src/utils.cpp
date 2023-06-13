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

#include "vslam_backend_plugins/utils.hpp"

namespace vslam_backend_plugins {
  namespace utils {
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::SE3Quat(R, t);
    }

    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::Sim3(R, t, scale);
    }
  }  // namespace utils
}  // namespace vslam_backend_plugins