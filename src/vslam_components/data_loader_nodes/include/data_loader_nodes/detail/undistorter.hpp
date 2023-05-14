// TODO: Licence

#ifndef DATA_LOADER_NODES__DETAIL__UNDISTORTER_HPP_
#define DATA_LOADER_NODES__DETAIL__UNDISTORTER_HPP_

#include <opencv2/core.hpp>

namespace vslam_components {
  namespace data_loader_nodes {
    namespace detail {
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

    }  // namespace detail
  }    // namespace data_loader_nodes
}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__DETAIL__UNDISTORTER_HPP_