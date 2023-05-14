#ifndef DATA_LOADER_NODES__CAMERA_INFO_HPP_
#define DATA_LOADER_NODES__CAMERA_INFO_HPP_

#include <opencv2/core.hpp>

namespace vslam_components {
  namespace data_loader_nodes {
    struct CameraInfo {
      // Focal length and principal point
      cv::Mat K;

      // distortion coefficient of shape (5, 1)
      // @sa https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
      cv::Mat dist_coeffs;

      // Image size
      int image_height;
      int image_width;
    };
  }  // namespace data_loader_nodes
}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__DETAIL__CAMERA_INFO_HPP_