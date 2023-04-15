#ifndef VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_

#include "vslam_datastructure/frame_pair.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_feature_matcher_base {
  class FeatureMatcher {
  public:
    virtual void initialize() = 0;
    virtual vslam_datastructure::Matches match_features(const vslam_datastructure::Points* const points1,
                                                        const vslam_datastructure::Points* const points2)
        = 0;
    vslam_datastructure::Point::Type point_type() const { return point_type_; };
    virtual ~FeatureMatcher() {}

  protected:
    FeatureMatcher() {}

    vslam_datastructure::Point::Type point_type_{vslam_datastructure::Point::Type::undefined};
  };
}  // namespace vslam_feature_matcher_base

#endif  // VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_