#ifndef VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_

#include "vslam_datastructure/frame_pair.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_feature_matcher_base {
  class FeatureMatcher {
  public:
    using FramePair = vslam_datastructure::FramePair;
    using Point = vslam_datastructure::Point;
    using PointType = vslam_datastructure::Point::Type;
    using Points = std::vector<Point>;
    using MatchedPoint = vslam_datastructure::MatchedPoint;
    using MatchedPoints = std::vector<MatchedPoint>;

    virtual void initialize() = 0;
    virtual MatchedPoints match_features(const Points& points1, const Points& points2) = 0;
    PointType point_type() const { return point_type_; };
    virtual ~FeatureMatcher() {}

  protected:
    FeatureMatcher() {}

    PointType point_type_{PointType::undefined};
  };
}  // namespace vslam_feature_matcher_base

#endif  // VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_