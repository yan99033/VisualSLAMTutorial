#ifndef VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_feature_extractor {
  namespace base {
    class FeatureExtractor : public virtual vslam_plugin::base::Plugin {
    public:
      /// Feature extractor initializer
      /**
       * \param num_features[in] maximum number of features to extract in an image
       * \param point_type[in] feature point type
       */
      virtual void initialize(const int num_features, const vslam_datastructure::Point::Type type) = 0;

      /// Extract features in the image
      /**
       * \param image[in] image
       * \return a vector of points
       */
      virtual vslam_datastructure::Points extractFeatures(const cv::Mat& image) = 0;

      /// Get the point type
      /**
       * \return point type
       */
      virtual inline vslam_datastructure::Point::Type pointType() const = 0;

      /// Destructor
      virtual ~FeatureExtractor() {}

    protected:
      /// Constructor
      FeatureExtractor() {}

      /// Point type
      vslam_datastructure::Point::Type point_type_{vslam_datastructure::Point::Type::undefined};
    };
  }  // namespace base
}  // namespace vslam_feature_extractor

#endif  // VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_