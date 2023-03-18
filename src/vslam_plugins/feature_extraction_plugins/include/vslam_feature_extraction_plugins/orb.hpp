#ifndef VSLAM_FEATURE_EXTRACTION_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_EXTRACTION_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/feature/orb.hpp"
#include "vslam_feature_extraction_plugins/feature_extraction_base.hpp"

namespace vslam_feature_extraction_plugins {
  class Orb : public vslam_feature_extraction_base::FeatureExtraction {
  public:
    using OrbFeature = vslam_datastructure::feature::Orb;

    void initialize(int num_features) override;

    Features extract_features(const cv::Mat& image) override;

    FeatureType feature_type() override { return feature_type_; }

  protected:
    int num_features_{1000};

  private:
    cv::Ptr<cv::ORB> orb_feature_detector_;
  };

}  // namespace vslam_feature_extraction_plugins

#endif
