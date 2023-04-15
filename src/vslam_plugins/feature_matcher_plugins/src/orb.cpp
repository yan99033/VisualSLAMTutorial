#include "vslam_feature_matcher_plugins/orb.hpp"

#include <cmath>
#include <iostream>

namespace {
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> extract_keypoints_descriptors(
      const vslam_datastructure::Points* points) {
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors_vec;
    for (const auto& pt : *points) {
      keypoints.push_back(pt->keypoint);
      descriptors_vec.push_back(pt->descriptor);
    }
    cv::Mat descriptors;
    cv::vconcat(descriptors_vec, descriptors);

    return {keypoints, descriptors};
  }

  vslam_datastructure::Matches create_matched_points(const vslam_datastructure::Points* points1,
                                                     const vslam_datastructure::Points* points2,
                                                     const std::vector<cv::DMatch>& matches,
                                                     const std::vector<char>& mask,
                                                     const double max_coord_dist = 100.0) {
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

        // Check distance
        if (cv::norm(points1->at(i1)->keypoint.pt - points2->at(i2)->keypoint.pt) > max_coord_dist) {
          continue;
        }

        // Create a match
        vslam_datastructure::MatchedPoint matched_point{points1->at(i1), points2->at(i2)};

        // Associate the map point with the corresponding point
        // if (points1->at(i1)->mappoint.get() && !points1->at(i1)->mappoint->is_outlier) {
        //   points2->at(i2)->mappoint = points1->at(i1)->mappoint;
        // }
        matched_points.push_back(matched_point);

        matched_index_pairs.push_back({i1, i2});
      }
    }

    return {matched_points, matched_index_pairs};
  }
}  // namespace

namespace vslam_feature_matcher_plugins {
  void Orb::initialize() {
    point_type_ = vslam_datastructure::Point::Type::orb;
    orb_feature_matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  }

  vslam_datastructure::Matches Orb::match_features(const vslam_datastructure::Points* const points1,
                                                   const vslam_datastructure::Points* const points2) {
    if (points1 == nullptr || points2 == nullptr) {
      vslam_datastructure::MatchedPoints();
    }

    // Get keypoints and descriptors
    auto [keypoints1, descriptors1] = extract_keypoints_descriptors(points1);
    auto [keypoints2, descriptors2] = extract_keypoints_descriptors(points2);

    // Match descriptors
    std::vector<cv::DMatch> matches;
    std::vector<char> matcher_mask;
    orb_feature_matcher_->match(descriptors1, descriptors2, matches, matcher_mask);

    const auto [matched_points, matched_index_pairs] = create_matched_points(points1, points2, matches, matcher_mask);

    return {matched_points, matched_index_pairs};
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::Orb, vslam_feature_matcher_base::FeatureMatcher)