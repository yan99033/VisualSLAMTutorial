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

#ifndef CAMERA_PLUGINS_BASE__MONOCULAR_HPP_
#define CAMERA_PLUGINS_BASE__MONOCULAR_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace camera_plugins {
  namespace base {
    class MonocularCamera {
    public:
      /// Camera initializer
      /**
       * Every camera has unique characteristics (e.g., focal lengths, principal point and distortion coefficient)
       * as well as a unique camera ID
       */
      virtual void initialize(const std::string& params) = 0;

      /// Grab the next image from the camera
      /**
       * \return An undistorted image
       */
      virtual cv::Mat grabImage() = 0;

      /// Get the camera matrix
      /**
       * \return Camera matrix
       */
      virtual cv::Mat K() = 0;

      /// Destructor
      virtual ~MonocularCamera() {}

    protected:
      /// Constructor
      MonocularCamera() {}
    };
  }  // namespace base
}  // namespace camera_plugins

#endif  // CAMERA_PLUGINS_BASE__MONOCULAR_HPP_