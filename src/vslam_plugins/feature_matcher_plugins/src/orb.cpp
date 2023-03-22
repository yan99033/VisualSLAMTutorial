#include "vslam_feature_matcher_plugins/orb.hpp"

#include <cmath>
#include <iostream>

namespace vslam_feature_matcher_plugins {
  void Orb::initialize() {
    point_type_ = PointType::orb;
    orb_feature_matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  }

  Orb::MatchedPoints Orb::match_features(const FramePair& frame_pair) {}

}  // namespace vslam_feature_matcher_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_matcher_plugins::Orb,
                       vslam_feature_matcher_base::FeatureMatcher)