#include "vslam_feature_extraction_plugins/orb.hpp"

#include <cmath>

namespace vslam_feature_extraction_plugins {

  void Orb::initialize(int num_features) {
    num_features_ = num_features;
    feature_type_ = OrbFeature::type;
    orb_feature_detector_ = cv::ORB::create(num_features);
  }

  Orb::Features Orb::extract_features(const cv::Mat& image) {
    // Extract features in the image
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_feature_detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  }

}  // namespace vslam_feature_extraction_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extraction_plugins::Orb,
                       vslam_feature_extraction_base::FeatureExtraction)
