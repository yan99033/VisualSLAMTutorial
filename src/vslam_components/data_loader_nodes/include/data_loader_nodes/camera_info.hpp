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

#ifndef DATA_LOADER_NODES__CAMERA_INFO_HPP_
#define DATA_LOADER_NODES__CAMERA_INFO_HPP_

#include <opencv2/core.hpp>

namespace vslam_components {
  namespace data_loader_nodes {
    struct CameraInfo {
      /// Camera matrix
      /**
       * [[fx,  0, cx],
       *  [ 0, fy, cy],
       *  [ 0,  0,  1]]
       */
      cv::Mat K;

      /// distortion coefficient
      /**
       * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
       */
      cv::Mat dist_coeffs;

      /// Image height
      int image_height;

      /// Image width
      int image_width;
    };
  }  // namespace data_loader_nodes
}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__DETAIL__CAMERA_INFO_HPP_