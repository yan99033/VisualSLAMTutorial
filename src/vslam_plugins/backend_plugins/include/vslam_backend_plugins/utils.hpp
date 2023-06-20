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

#ifndef VSLAM_BACKEND_PLUGINS__UTILS_HPP_
#define VSLAM_BACKEND_PLUGINS__UTILS_HPP_

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <opencv2/opencv.hpp>
#include <unordered_set>

#include "vslam_datastructure/frame.hpp"

namespace vslam_backend_plugins {
  namespace utils {
    /// Convert a 4x4 SE3 transformation matrix to g2o SE3Quat
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose);

    /// Convert a 4x4 SE3 transformation matrix plus a scale to g2o Sim3
    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale);

    /// Get the nearby (key)frames based on the map point projections
    /**
     * \param frame the keyframe
     * \param min_projections minumum number of projections required to be considered as a nearby keyframe
     * \param top_k_projections top k keyframes ranked by the projection score, where the score is the number
     * of projections divided by the keyframe id distance
     * \param use_host_mps use the map points that the frame hosts when calculating projections
     * \return the top k keyframes
     */
    std::unordered_set<const vslam_datastructure::Frame*> getFrameMappointProjectedFrames(
        vslam_datastructure::Frame* const frame, const bool use_host_mps = false, const size_t min_projections = 10,
        const size_t top_k_projections = 5);

  }  // namespace utils
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__UTILS_HPP_