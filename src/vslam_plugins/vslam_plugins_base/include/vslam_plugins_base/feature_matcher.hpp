#ifndef VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_feature_matcher {
  namespace base {
    class FeatureMatcher : public virtual vslam_plugin::base::Plugin {
    public:
      /// Feature matcher initializer
      virtual void initialize(const vslam_datastructure::Point::Type point_type) = 0;

      /// Match features between two sets of points
      /**
       * \param[in] points1 a vector of points from frame 1
       * \param[in] points2 a vector of points from frame 2
       * \return matches between the two vector of points
       */
      virtual vslam_datastructure::Matches matchFeatures(const vslam_datastructure::Points& points1,
                                                         const vslam_datastructure::Points& points2)
          = 0;

      /// Destructor
      virtual ~FeatureMatcher() {}

    protected:
      /// Constructor
      FeatureMatcher() {}
    };
  }  // namespace base
}  // namespace vslam_feature_matcher

#endif  // VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_