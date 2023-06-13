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

#ifndef VSLAM_BACKEND_PLUGINS__UTILS_HPP_
#define VSLAM_BACKEND_PLUGINS__UTILS_HPP_

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <opencv2/opencv.hpp>

namespace vslam_backend_plugins {
  namespace utils {
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose);

    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale);

  }  // namespace utils
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__UTILS_HPP_