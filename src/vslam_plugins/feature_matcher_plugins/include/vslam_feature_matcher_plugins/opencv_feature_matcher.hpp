#ifndef VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"

namespace vslam_feature_matcher_plugins {

  class OpenCVFeatureMatcher : public virtual vslam_feature_matcher::base::FeatureMatcher {
  public:
    ~OpenCVFeatureMatcher() { std::cerr << "Terminated OpenCVFeatureMatcher" << std::endl; }

    void initialize(const vslam_datastructure::Point::Type point_type) override;

    vslam_datastructure::Matches match_features(const vslam_datastructure::Points& points1,
                                                const vslam_datastructure::Points& points2) override;

    inline std::string get_plugin_name() override { return "vslam_feature_matcher_plugins::OpenCVFeatureMatcher"; }

  private:
    cv::Ptr<cv::DescriptorMatcher> feature_matcher_;
  };
}  // namespace vslam_feature_matcher_plugins

#endif  // VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
