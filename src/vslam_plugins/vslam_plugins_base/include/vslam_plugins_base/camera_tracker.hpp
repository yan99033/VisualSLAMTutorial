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

#ifndef VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_
#define VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_camera_tracker {
  namespace base {
    class CameraTracker : public virtual vslam_plugin::base::Plugin {
    public:
      /// Camera tracker initializer
      virtual void initialize() = 0;

      /// Track the relative transformation (T_2_1) given a vector containing the 2D-2D correspondences between frame1
      /// and frame2
      /**
       * The tracking quality is indicated by the number of inlier points relative to the total amount of
       * correspondences provided
       *
       * \param[in] matched_points a vector containing the 2D-2D correspondences between frame1 and frame2
       * \param[out] T_2_1 the camera transformation from frame2 to frame1
       * \return a boolean indicating the tracking quality
       */
      virtual bool trackCamera2d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                   cv::Mat& T_2_1)
          = 0;

      /// Track the relative transformation (T_2_1) given a vector containing the 3D-2D correspondences between frame1
      /// and frame2
      /**
       * The tracking quality is indicated by the number of inlier points relative to the total amount of
       * correspondences provided
       *
       * \param[in] matched_points a vector containing the 3D-2D correspondences between frame1 and frame2
       * \param[out] T_2_1 the camera transformation from frame2 to frame1
       * \return a boolean indicating the tracking quality
       */
      virtual bool trackCamera3d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                   cv::Mat& T_2_1)
          = 0;

      /// Destructor
      virtual ~CameraTracker() {}

    protected:
      /// Constructor
      CameraTracker() {}
    };
  }  // namespace base
}  // namespace vslam_camera_tracker

#endif  // VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_