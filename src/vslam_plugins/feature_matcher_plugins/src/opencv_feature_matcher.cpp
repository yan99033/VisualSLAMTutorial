#include "vslam_feature_matcher_plugins/opencv_feature_matcher.hpp"

#include <cmath>
#include <iostream>

namespace {
  /// Concatenate the keypoint descriptors into a matrix
  /**
   * \param points[in] a vector of points
   * \return a pair of type keypoints and descriptors
   */
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> extractKeypointsDescriptors(const vslam_datastructure::Points& points) {
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors_vec;
    for (const auto& pt : points) {
      keypoints.push_back(pt->keypoint);
      descriptors_vec.push_back(pt->descriptor);
    }
    cv::Mat descriptors;
    cv::vconcat(descriptors_vec, descriptors);

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
  vslam_datastructure::Matches createMatchedPoints(const vslam_datastructure::Points& points1,
                                                   const vslam_datastructure::Points& points2,
                                                   const std::vector<cv::DMatch>& matches,
                                                   const std::vector<char>& mask) {
    if (!mask.empty() && (mask.size() != matches.size())) {
      throw std::runtime_error("Invalid mask " + std::to_string(mask.size()) + " or matches size "
                               + std::to_string(matches.size()));
    }

    vslam_datastructure::MatchedPoints matched_points;
    vslam_datastructure::MatchedIndexPairs matched_index_pairs;
    for (size_t i = 0; i < matches.size(); i++) {
      if (mask.empty() || mask[i]) {
        const int i1 = matches[i].queryIdx;
        const int i2 = matches[i].trainIdx;

        // Create a match
        vslam_datastructure::MatchedPoint matched_point{points1.at(i1), points2.at(i2)};

        matched_points.push_back(matched_point);

        matched_index_pairs.push_back({i1, i2});
      }
    }

    return {matched_points, matched_index_pairs};
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

  vslam_datastructure::Matches OpenCVFeatureMatcher::matchFeatures(const vslam_datastructure::Points& points1,
                                                                   const vslam_datastructure::Points& points2) {
    if (points1.empty() || points2.empty()) {
      return {vslam_datastructure::MatchedPoints(), vslam_datastructure::MatchedIndexPairs()};
    }

    // Get keypoints and descriptors
    auto [keypoints1, descriptors1] = extractKeypointsDescriptors(points1);
    auto [keypoints2, descriptors2] = extractKeypointsDescriptors(points2);

    // Match descriptors
    std::vector<cv::DMatch> matches;
    std::vector<char> matcher_mask;
    feature_matcher_->match(descriptors1, descriptors2, matches, matcher_mask);

    const auto [matched_points, matched_index_pairs] = createMatchedPoints(points1, points2, matches, matcher_mask);

    return {matched_points, matched_index_pairs};
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::OpenCVFeatureMatcher, vslam_feature_matcher::base::FeatureMatcher)