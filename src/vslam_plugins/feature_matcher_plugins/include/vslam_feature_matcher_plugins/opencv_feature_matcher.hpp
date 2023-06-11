#ifndef VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"

namespace vslam_feature_matcher_plugins {

  class OpenCVFeatureMatcher : public virtual vslam_feature_matcher::base::FeatureMatcher {
  public:
    /// Destructor
    ~OpenCVFeatureMatcher() { std::cerr << "Terminated OpenCVFeatureMatcher" << std::endl; }

    /// Feature matcher initializer
    void initialize(const vslam_datastructure::Point::Type point_type) override;

    /// Match features between two sets of points
    /**
     * \param[in] points1 a vector of points from frame 1
     * \param[in] points2 a vector of points from frame 2
     * \return matches between the two vector of points
     */
    vslam_datastructure::Matches matchFeatures(const vslam_datastructure::Points& points1,
                                               const vslam_datastructure::Points& points2) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_feature_matcher_plugins::OpenCVFeatureMatcher"; }

  private:
    cv::Ptr<cv::DescriptorMatcher> feature_matcher_;
  };
}  // namespace vslam_feature_matcher_plugins

#endif  // VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
