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
       * \param image[in] a greyscale image
       * \return a vector of keypoints
       */
      virtual KeyPoints calculateKeypoints(const cv::Mat& grey_image) = 0;
    };
  }  // namespace abstract

  class OpenCVFeatureExtractor : public virtual abstract::OpenCVFeatureExtractor {
  public:
    ~OpenCVFeatureExtractor() { std::cerr << "Terminated OpenCVFeatureExtractor" << std::endl; }

    /// Initialize feature extractor
    /**
     * \param num_features[in] maximum number of features to extract in an image
     * \param point_type[in] feature point type
     */
    void initialize(const int num_feature, const vslam_datastructure::Point::Type point_type) override;

    /// Extract features in the image
    /**
     * \param image[in] image
     * \return a vector of points
     */
    vslam_datastructure::Points extractFeatures(const cv::Mat& image) override;

    /// Get the point type
    /**
     * \return point type
     */
    inline vslam_datastructure::Point::Type pointType() const override { return point_type_; };

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_feature_extractor_plugins::OpenCVFeatureExtractor"; }

  protected:
    /// Maximum number of features to extract in an image
    int num_features_{1000};

    /// Feature detector
    /**
     * \sa https://docs.opencv.org/4.x/d0/d13/classcv_1_1Feature2D.html
     */
    cv::Ptr<cv::Feature2D> feature_detector_{nullptr};

    /// Calculate the 2D keypoints in the image
    /**
     * \sa https://docs.opencv.org/4.x/d2/d29/classcv_1_1KeyPoint.html
     * \param image[in] a greyscale image
     * \return a vector of keypoints
     */
    KeyPoints calculateKeypoints(const cv::Mat& image) override;

  private:
    using CvMatPyr = std::vector<cv::Mat>;

    /// Parameter characterizing the minimal accepted quality of image corners
    /**
     * \sa https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541
     */
    static constexpr const double quality_level_{0.01};

    /// 	Minimum possible Euclidean distance between the returned corners
    /**
     * \sa https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541
     */
    static constexpr const double min_dist_{10};

    /// Number of pyramid levels
    static constexpr const size_t nlevels_{8};

    /// Pyramid decimation ratio
    static constexpr const double scale_factor_{1.2};

    /// Size of the patch for calculating Harris score
    static constexpr const int harris_block_size_{7};

    /// Size of the patch used by the oriented BRIEF descriptor
    static constexpr const int patch_size_{31};
  };

}  // namespace vslam_feature_extractor_plugins

#endif  // VSLAM_FEATURE_EXTRACTOR_PLUGINS__OPENCV_FEATURE_EXTRACTOR_HPP_