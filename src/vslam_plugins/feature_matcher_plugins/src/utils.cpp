#include "vslam_feature_matcher_plugins/utils.hpp"

namespace vslam_feature_matcher_plugins {

  std::pair<std::vector<cv::KeyPoint>, cv::Mat> extract_keypoints_descriptors(
      const vslam_datastructure::Points& points) {
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

  vslam_datastructure::MatchedPoints create_matched_points(
      const vslam_datastructure::Points& points1, const vslam_datastructure::Points& points2,
      const std::vector<cv::DMatch>& matches, const std::vector<char>& mask) {
    if (!mask.empty() && (mask.size() != matches.size())) {
      throw std::runtime_error("Invalid mask " + std::to_string(mask.size()) + " or matches size "
                               + std::to_string(matches.size()));
    }

    vslam_datastructure::MatchedPoints matched_points;
    for (size_t i = 0; i < matches.size(); i++) {
      if (mask.empty() || mask[i]) {
        const int i1 = matches[i].queryIdx;
        const int i2 = matches[i].trainIdx;

        vslam_datastructure::MatchedPoint matched_point{points1[i1], points2[i2]};
        matched_points.push_back(matched_point);
      }
    }

    return matched_points;
  }

}  // namespace vslam_feature_matcher_plugins