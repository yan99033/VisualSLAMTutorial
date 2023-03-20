#include "vslam_feature_extractor_plugins/orb.hpp"

#include <cmath>

namespace vslam_feature_extractor_plugins {
  class Orb : public vslam_feature_extractor_base::FeatureExtractor {
  public:
    using OrbFeature = vslam_datastructure::feature::Orb;

    void initialize(int num_features) override {
      num_features_ = num_features;
      feature_type_ = OrbFeature::type;
      orb_feature_detector_ = cv::ORB::create(num_features);
    }

    Features extract_features(const cv::Mat& image) override {
      // Extract features in the image
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      orb_feature_detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

      Features orb_features;
      for (std::ptrdiff_t i = 0; i < keypoints.size(); i++) {
        OrbFeature orb_feature;
        orb_feature.keypoint = keypoints[i];
        std::copy(descriptors.row(i).data, descriptors.row(i).data + orb_feature.desc_len,
                  orb_feature.descriptor);
        orb_features.push_back(orb_feature);
      }
      return orb_features;
    }

    FeatureType feature_type() override { return feature_type_; }

  protected:
    int num_features_{1000};

  private:
    cv::Ptr<cv::ORB> orb_feature_detector_;
  };

}  // namespace vslam_feature_extractor_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extractor_plugins::Orb,
                       vslam_feature_extractor_base::FeatureExtractor)