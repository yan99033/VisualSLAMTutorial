#ifndef DATA_LOADER_NODES__CAMERA_INFO_HPP_
#define DATA_LOADER_NODES__CAMERA_INFO_HPP_

#include <opencv2/core.hpp>

namespace vslam_components {
  namespace data_loader_nodes {
    struct CameraInfo {
      /// Camera matrix
      /**
       * [[fx,  0, cx],
       *  [ 0, fy, cy],
       *  [ 0,  0,  1]]
       */
      cv::Mat K;

      /// distortion coefficient
      /**
       * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
       */
      cv::Mat dist_coeffs;

      /// Image height
      int image_height;

      /// Image width
      int image_width;
    };
  }  // namespace data_loader_nodes
}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__DETAIL__CAMERA_INFO_HPP_