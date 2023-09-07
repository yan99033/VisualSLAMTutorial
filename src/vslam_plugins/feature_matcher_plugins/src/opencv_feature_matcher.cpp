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

#include "vslam_feature_matcher_plugins/opencv_feature_matcher.hpp"

#include <cmath>
#include <iostream>

#include "vslam_datastructure/utils.hpp"

namespace {
  /// Concatenate the keypoint descriptors into a matrix
  /**
   * \param points[in] a vector of points
   * \return a pair of type keypoints and descriptors
   */
  std::pair<vslam_datastructure::KeyPoints, cv::Mat> extractKeypointsDescriptors(
      const vslam_datastructure::Points& points) {
    const auto keypoints = vslam_datastructure::utils::extractKeypointsFromPoints(points);
    const auto descriptors = vslam_datastructure::utils::extractDescriptorsFromPoints(points);

    return {keypoints, descriptors};
  }

  /// Create matched points and indices from the match results
  /**
   * \param[in] points1 a vector of points from frame 1
   * \param[in] points2 a vector of points from frame 2
   * \param[in] matches match results
   * \param[in] mask the inlier mask of the match results
   * \return matches which consists of point correspondences and the corresponding indices
   */
  vslam_datastructure::MatchedPoints createMatchedPoints(const vslam_datastructure::Points& points1,
                                                         const vslam_datastructure::Points& points2,
                                                         const std::vector<cv::DMatch>& matches,
                                                         const std::vector<char>& mask) {
    if (!mask.empty() && (mask.size() != matches.size())) {
      throw std::runtime_error("Invalid mask " + std::to_string(mask.size()) + " or matches size "
                               + std::to_string(matches.size()));
    }

    vslam_datastructure::MatchedPoints matched_points;
    for (size_t i = 0; i < matches.size(); i++) {
      if (mask.empty() || mask[i]) {
        const int i1 = matches[i].queryIdx;
        const int i2 = matches[i].trainIdx;

        // Create a match
        vslam_datastructure::MatchedPoint matched_point{points1.at(i1), points2.at(i2)};

        matched_points.push_back(matched_point);
      }
    }

    return matched_points;
  }
}  // namespace

namespace vslam_feature_matcher_plugins {
  void OpenCVFeatureMatcher::initialize(const vslam_datastructure::Point::Type point_type) {
    switch (point_type) {
      case vslam_datastructure::Point::Type::orb:
        feature_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        break;
      default:
        throw std::runtime_error("OpenCVFeatureMatcher::initialize: feature matcher is not defined.");
    }
  }

  vslam_datastructure::MatchedPoints OpenCVFeatureMatcher::matchFeatures(const vslam_datastructure::Points& points1,
                                                                         const vslam_datastructure::Points& points2) {
    if (points1.empty() || points2.empty()) {
      return vslam_datastructure::MatchedPoints();
    }

    // Get keypoints and descriptors
    auto [keypoints1, descriptors1] = extractKeypointsDescriptors(points1);
    auto [keypoints2, descriptors2] = extractKeypointsDescriptors(points2);

    // Match descriptors
    std::vector<cv::DMatch> matches;
    std::vector<char> matcher_mask;
    feature_matcher_->match(descriptors1, descriptors2, matches, matcher_mask);

    const auto matched_points = createMatchedPoints(points1, points2, matches, matcher_mask);

    return matched_points;
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::OpenCVFeatureMatcher, vslam_feature_matcher::base::FeatureMatcher)