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

#include "vslam_mapper_plugins/opencv_mapper.hpp"

#include <cmath>
#include <iostream>

namespace {
  /// Preprocess the correspondences for triangulations
  /**
   * \param[in] matched_points point correspondences
   * \return a pair of type cv::Mat containing the keypoints
   */
  std::pair<cv::Mat, cv::Mat> getCorrespondences(const vslam_datastructure::MatchedPoints& matched_points) {
    const size_t npts = matched_points.size();
    cv::Mat points1_mat = cv::Mat(2, npts, CV_64F);
    cv::Mat points2_mat = cv::Mat(2, npts, CV_64F);

    for (size_t i = 0; i < matched_points.size(); i++) {
      points1_mat.at<double>(0, i) = matched_points[i].point1->keypoint.pt.x;
      points1_mat.at<double>(1, i) = matched_points[i].point1->keypoint.pt.y;
      points2_mat.at<double>(0, i) = matched_points[i].point2->keypoint.pt.x;
      points2_mat.at<double>(1, i) = matched_points[i].point2->keypoint.pt.y;
    }

    return {points1_mat, points2_mat};
  }

  /// Project map point to the camera frame
  /**
   * \param[in] 3D position of the map point
   * \param[in] camera matrix
   * \return projected 2D image coordinate
   */
  cv::Mat projectMappoint3d2d(const cv::Mat& pos, const cv::Mat& K) {
    cv::Mat pos_projected = pos.clone();
    pos_projected = K * pos_projected;
    pos_projected /= pos_projected.at<double>(2, 0);

    return pos_projected.rowRange(0, 2);
  }
}  // namespace

namespace vslam_mapper_plugins {
  vslam_datastructure::MapPoints OpenCVMapper::map(vslam_datastructure::MatchedPoints& matched_points,
                                                   const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K) {
    // Preprocess the matched points for triangulation
    const auto [cv_points1, cv_points2] = getCorrespondences(matched_points);

    // Calculate the projection matrix
    cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
    P1 = K * P1.rowRange(0, 3).colRange(0, 4);
    cv::Mat P2 = K * T_2_1.rowRange(0, 3).colRange(0, 4);

    // Triangulate the map points
    cv::Mat cv_mappoints1_3d;
    cv::triangulatePoints(P1, P2, cv_points1, cv_points2, cv_mappoints1_3d);

    // Create map points for the points
    vslam_datastructure::MapPoints new_mps;
    for (size_t i = 0; i < matched_points.size(); i++) {
      // Normalize and transform to the global coordinates
      auto pos_3d = cv_mappoints1_3d.col(i);
      pos_3d /= pos_3d.at<double>(3, 0);

      // Check if NaN exists
      bool nan_found{false};
      for (size_t j = 0; j < 3; j++) {
        if (std::isnan(pos_3d.at<double>(j, 0))) {
          nan_found = true;
          break;
        }
      }
      if (nan_found) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Remove points that have a large re-projection error
      cv::Mat pos_3d_projected = projectMappoint3d2d(pos_3d.rowRange(0, 3), K);
      if (cv::norm(pos_3d_projected, cv_points1.col(i)) > proj_err_thresh_) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Remove points behind the camera
      if (pos_3d.at<double>(2, 0) <= 0.0) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Transform to world coordinate
      pos_3d = T_1_w.inv() * pos_3d;

      // Create a new map point
      vslam_datastructure::MapPoint::SharedPtr mp = std::make_shared<vslam_datastructure::MapPoint>();
      mp->setPos(cv::Point3d(pos_3d.at<double>(0, 0), pos_3d.at<double>(1, 0), pos_3d.at<double>(2, 0)));

      new_mps.push_back(mp);
    }

    return new_mps;
  }

}  // namespace vslam_mapper_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_mapper_plugins::OpenCVMapper, vslam_mapper::base::Mapper)