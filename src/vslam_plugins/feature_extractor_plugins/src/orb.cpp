#include "vslam_feature_extractor_plugins/orb.hpp"

#include <cmath>
#include <iostream>

namespace vslam_feature_extractor_plugins {
  void Orb::initialize(int num_features) {
    num_features_ = num_features;
    point_type_ = vslam_datastructure::PointType::orb;
    orb_feature_detector_ = cv::ORB::create(num_features);
  }

  vslam_datastructure::Points Orb::extract_features(const cv::Mat& image) {
    // Extract features in the image
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_feature_detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

    vslam_datastructure::Points orb_ft_points;
    for (std::ptrdiff_t i = 0; i < keypoints.size(); i++) {
      auto pt = std::make_shared<vslam_datastructure::Point>();
      pt->keypoint = keypoints[i];
      pt->descriptor = descriptors.row(i);
      pt->type = point_type_;
      orb_ft_points.push_back(pt);
    }
    return orb_ft_points;
  }

}  // namespace vslam_feature_extractor_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extractor_plugins::Orb,
                       vslam_feature_extractor_base::FeatureExtractor)