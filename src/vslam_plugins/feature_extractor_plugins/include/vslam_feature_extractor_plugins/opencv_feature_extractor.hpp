#ifndef VSLAM_FEATURE_EXTRACTOR_PLUGINS__OPENCV_FEATURE_EXTRACTOR_HPP_
#define VSLAM_FEATURE_EXTRACTOR_PLUGINS__OPENCV_FEATURE_EXTRACTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

#include "vslam_plugins_base/feature_extractor.hpp"

namespace vslam_feature_extractor_plugins {
  namespace abstract {
    class OpenCVFeatureExtractor : public virtual vslam_feature_extractor::base::FeatureExtractor {
    protected:
      using KeyPoints = std::vector<cv::KeyPoint>;

      /// Calculate the 2D keypoints in the image
      /**
       * \sa https://docs.opencv.org/4.x/d2/d29/classcv_1_1KeyPoint.html
       * \param image a greyscale image
       * \return a vector of keypoints
       */
      /// @param image
      /// @return a vector of 2D keypoints
      virtual KeyPoints calculate_keypoints(const cv::Mat& grey_image) = 0;
    };
  }  // namespace abstract

  class OpenCVFeatureExtractor : public virtual abstract::OpenCVFeatureExtractor {
  public:
    void initialize(int num_feature, const vslam_datastructure::Point::Type point_type) override;

    vslam_datastructure::Points extract_features(const cv::Mat& image) override;

  protected:
    /// Maximum number of features to extract in an image
    int num_features_{1000};

    cv::Ptr<cv::Feature2D> feature_detector_{nullptr};

    KeyPoints calculate_keypoints(const cv::Mat& image) override;

  private:
    using CvMatPyr = std::vector<cv::Mat>;

    // goodFeaturesToTrack
    static constexpr const double quality_level_{0.01};
    static constexpr const double min_dist_{10};

    // Keypoint extractor
    static constexpr const size_t nlevels_{8};
    static constexpr const double scale_factor_{1.2};
    static constexpr const int harris_block_size_{7};
    static constexpr const int patch_size_{31};
    static constexpr const int desc_size_{32};
  };

}  // namespace vslam_feature_extractor_plugins

#endif  // VSLAM_FEATURE_EXTRACTOR_PLUGINS__OPENCV_FEATURE_EXTRACTOR_HPP_