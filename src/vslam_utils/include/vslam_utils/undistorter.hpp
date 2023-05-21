// TODO: Licence

#ifndef VSLAM_UTILS__UNDISTORTER_HPP_
#define VSLAM_UTILS__UNDISTORTER_HPP_

#include <opencv2/opencv.hpp>

namespace vslam_utils {
  class Undistorter {
  public:
    // Given the camera matrix and distortion coefficients,
    // Undistorter calculates the new camera matrix and undistorted image
    Undistorter(const cv::Mat& K, const int image_width, const int image_height,
                const cv::Mat& dist_coeffs = cv::Mat());

    cv::Mat get_K() const;

    cv::Mat undistort_image(const cv::Mat& in_image) const;

  private:
    // If no distortion coefficient is provided, we just pass the image and camera matrix through
    bool passthrough_{true};

    cv::Mat K_;

    cv::Mat map1_;
    cv::Mat map2_;
  };

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__UNDISTORTER_HPP_