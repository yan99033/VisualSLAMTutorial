#include "vslam_utils/undistorter.hpp"

namespace vslam_utils {
  Undistorter::Undistorter(const cv::Mat& K, const int image_width, const int image_height,
                           const cv::Mat& dist_coeffs) {
    if (dist_coeffs.empty() || cv::sum(dist_coeffs) == cv::Scalar(0.0)) {
      K_ = K.clone();
      return;
    }

    passthrough_ = false;

    K_ = cv::getOptimalNewCameraMatrix(K, dist_coeffs, cv::Size(image_width, image_height), 0,
                                       cv::Size(image_width, image_height), nullptr, false);

    cv::initUndistortRectifyMap(K, dist_coeffs, cv::Mat(), K_, cv::Size(image_width, image_height), CV_16SC2, map1_,
                                map2_);
  }

  cv::Mat Undistorter::get_K() const { return K_.clone(); }

  cv::Mat Undistorter::undistort_image(const cv::Mat& in_image) const {
    if (passthrough_) {
      return in_image;
    }

    cv::Mat out_image;
    cv::remap(in_image, out_image, map1_, map2_, cv::INTER_LINEAR);

    return out_image;
  }

}  // namespace vslam_utils