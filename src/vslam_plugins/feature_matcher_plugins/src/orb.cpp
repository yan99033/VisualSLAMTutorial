#include "vslam_feature_matcher_plugins/orb.hpp"

#include <cmath>
#include <iostream>

namespace vslam_feature_matcher_plugins {
  void Orb::initialize() {
    point_type_ = PointType::orb;
    orb_feature_matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  }

  Orb::MatchedPoints Orb::match_features(const Points& points1, const Points& points2) {
    //   std::vector<cv::KeyPoint> keypoints;
    //     std::vector<cv::Mat> descriptors_vec;
    //     for (const auto &ft : points) {
    //       keypoints.push_back(ft.keypoint);
    //       descriptors_vec.push_back(ft.descriptor);
    //     }
    //     cv::Mat descriptors;
    //     cv::vconcat(descriptors_vec, descriptors);
  }

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::Orb,
                       vslam_feature_matcher_base::FeatureMatcher)