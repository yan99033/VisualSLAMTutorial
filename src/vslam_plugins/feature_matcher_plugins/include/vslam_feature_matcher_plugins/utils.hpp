#ifndef VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_
#define VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_

#include <opencv2/core.hpp>

#include "vslam_datastructure/point.hpp"

namespace vslam_feature_matcher_plugins {
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> extract_keypoints_descriptors(
      const vslam_datastructure::Points& points);

  vslam_datastructure::MatchedPoints create_matched_points(
      const vslam_datastructure::Points& points1, const vslam_datastructure::Points& points2,
      const std::vector<cv::DMatch>& matches, const std::vector<char>& matcher_mask);

}  // namespace vslam_feature_matcher_plugins

#endif  // VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_