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

#include "vslam_feature_matcher_plugins/optical_flow_feature_matcher.hpp"

#include <cmath>
#include <iostream>

namespace {
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
                                                         const std::vector<cv::Point2f>& matches,
                                                         const std::vector<uchar>& mask) {
    if (!mask.empty() && (mask.size() != matches.size())) {
      throw std::runtime_error("Invalid mask " + std::to_string(mask.size()) + " or matches size "
                               + std::to_string(matches.size()));
    }

    // Find the closest match in points2
    constexpr const double dist_thresh{1.0};
    std::unordered_map<size_t, std::pair<size_t, double>> point_idx_matches;
    for (size_t i = 0; i < matches.size(); i++) {
      if (mask[i]) {
        for (size_t j = 0; j < points2.size(); j++) {
          // Calculate the distance
          const double dist = cv::norm(matches[i] - points2[j]->keypoint.pt);

          // if distance is smaller than dist_thresh (consider a match)
          if (dist < dist_thresh) {
            // Check if there is an existing match and replace the match if the distance is smaller
            if (point_idx_matches.find(j) != point_idx_matches.end()) {
              const auto [_, prev_dist] = point_idx_matches[j];

              if (dist < prev_dist) {
                point_idx_matches[j] = {i, dist};
              }
            } else {
              point_idx_matches[j] = {i, dist};
            }
          }
        }
      }
    }

    vslam_datastructure::MatchedPoints matched_points;
    for (const auto& match : point_idx_matches) {
      const auto i2 = match.first;
      const auto i1 = match.second.first;

      // If the matched point does not have a map point, replace the coordinate of the keypoint
      if (!points2.at(i2)->hasMappoint()) {
        points2.at(i2)->keypoint.pt = matches.at(i1);
      }

      // Create a match
      vslam_datastructure::MatchedPoint matched_point{points1.at(i1), points2.at(i2)};

      matched_points.push_back(matched_point);
    }

    return matched_points;
  }
}  // namespace

namespace vslam_feature_matcher_plugins {
  void LKOpticalFlowFeatureMatcher::initialize(const vslam_datastructure::Point::Type point_type) { (void)point_type; }

  vslam_datastructure::MatchedPoints LKOpticalFlowFeatureMatcher::matchFeatures(
      const vslam_datastructure::Points& points1, const vslam_datastructure::Points& points2) {
    if (points1.empty() || points2.empty()) {
      return vslam_datastructure::MatchedPoints();
    }

    // Frame pointer
    const auto frame1 = points1[0]->frame();
    const auto frame2 = points2[0]->frame();

    // Extract keypoints
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2_matches;
    for (const auto& pt1 : points1) {
      pts1.emplace_back(pt1->keypoint.pt);
      pts2_matches.emplace_back(pt1->keypoint.pt);
    }

    // Match points
    std::vector<uchar> matcher_mask;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        frame1->image(), frame2->image(), pts1, pts2_matches, matcher_mask, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

    const auto matched_points = createMatchedPoints(points1, points2, pts2_matches, matcher_mask);

    return matched_points;
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::LKOpticalFlowFeatureMatcher,
                       vslam_feature_matcher::base::FeatureMatcher)