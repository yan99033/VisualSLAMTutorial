#ifndef VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_
#define VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_

#include <opencv2/core.hpp>

#include "vslam_datastructure/point.hpp"

namespace vslam_feature_matcher_plugins {
  using MatchedPoint = vslam_datastructure::MatchedPoint;
  using MatchedPoints = std::vector<MatchedPoint>;
  using Point = vslam_datastructure::Point;
  using Points = std::vector<Point>;

  std::pair<std::vector<cv::KeyPoint>, cv::Mat> extract_keypoints_descriptors(const Points& points);

  MatchedPoints create_matched_points(const Points& points1, const Points& points2,
                                      const std::vector<cv::DMatch>& matches,
                                      const std::vector<char>& matcher_mask);

}  // namespace vslam_feature_matcher_plugins

#endif  // VSLAM_FEATURE_MATCHER_PLUGINS__UTILS_HPP_