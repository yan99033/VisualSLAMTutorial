#include "vslam_feature_matcher_plugins/orb.hpp"

#include <cmath>
#include <iostream>

#include "vslam_feature_matcher_plugins/utils.hpp"

namespace vslam_feature_matcher_plugins {
  void Orb::initialize() {
    point_type_ = vslam_datastructure::PointType::orb;
    orb_feature_matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  }

  vslam_datastructure::MatchedPoints Orb::match_features(
      const vslam_datastructure::Points& points1, const vslam_datastructure::Points& points2) {
    // Get keypoints and descriptors
    auto [keypoints1, descriptors1] = extract_keypoints_descriptors(points1);
    auto [keypoints2, descriptors2] = extract_keypoints_descriptors(points2);

    // Match descriptors
    std::vector<cv::DMatch> matches;
    std::vector<char> matcher_mask;
    orb_feature_matcher_->match(descriptors1, descriptors2, matches, matcher_mask);

    auto matched_points = create_matched_points(points1, points2, matches, matcher_mask);

    return matched_points;
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::Orb,
                       vslam_feature_matcher_base::FeatureMatcher)