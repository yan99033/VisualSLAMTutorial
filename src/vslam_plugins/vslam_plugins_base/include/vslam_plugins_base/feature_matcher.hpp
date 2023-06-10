#ifndef VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_feature_matcher {
  namespace base {
    class FeatureMatcher : public virtual vslam_plugin::base::Plugin {
    public:
      virtual void initialize(const vslam_datastructure::Point::Type point_type) = 0;
      virtual vslam_datastructure::Matches match_features(const vslam_datastructure::Points& points1,
                                                          const vslam_datastructure::Points& points2)
          = 0;
      virtual ~FeatureMatcher() {}

    protected:
      FeatureMatcher() {}
    };
  }  // namespace base
}  // namespace vslam_feature_matcher

#endif  // VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_