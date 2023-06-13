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

#include "vslam_utils/converter.hpp"

namespace vslam_utils {
  namespace conversions {
    cv::Mat toTransformationMatrix(const cv::Mat& R, const cv::Mat& t) {
      // Create an identity matrix
      cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

      // Copy data
      R.copyTo(T(cv::Rect(0, 0, 3, 3)));
      t.copyTo(T(cv::Rect(3, 0, 1, 3)));

      return T;
    }
  }  // namespace conversions
}  // namespace vslam_utils