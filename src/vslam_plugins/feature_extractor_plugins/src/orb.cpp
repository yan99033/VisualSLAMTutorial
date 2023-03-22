#include "vslam_feature_extractor_plugins/orb.hpp"

#include <cmath>
#include <iostream>

namespace vslam_feature_extractor_plugins {
  class Orb : public vslam_feature_extractor_base::FeatureExtractor {
  public:
    void initialize(int num_features) override {
      num_features_ = num_features;
      point_type_ = PointType::orb;
      orb_feature_detector_ = cv::ORB::create(num_features);
    }

    Points extract_features(const cv::Mat& image) override {
      // Extract features in the image
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      orb_feature_detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

      Points orb_ft_points;
      for (std::ptrdiff_t i = 0; i < keypoints.size(); i++) {
        Point pt;
        pt.keypoint = keypoints[i];
        pt.descriptor = descriptors.row(i);
        pt.type = point_type_;
        orb_ft_points.push_back(pt);
      }
      return orb_ft_points;
    }

    PointType point_type() override { return point_type_; }

  protected:
    int num_features_{1000};

  private:
    cv::Ptr<cv::ORB> orb_feature_detector_;
  };

}  // namespace vslam_feature_extractor_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extractor_plugins::Orb,
                       vslam_feature_extractor_base::FeatureExtractor)