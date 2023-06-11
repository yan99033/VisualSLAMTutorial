#ifndef CAMERA_PLUGINS_BASE__MONOCULAR_HPP_
#define CAMERA_PLUGINS_BASE__MONOCULAR_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace camera_plugins {
  namespace base {
    class MonocularCamera {
    public:
      /// Camera initializer
      /**
       * Every camera has unique characteristics (e.g., focal lengths, principal point and distortion coefficient)
       * as well as a unique camera ID
       */
      virtual void initialize(const std::string& params) = 0;

      /// Grab the next image from the camera
      /**
       * \return An undistorted image
       */
      virtual cv::Mat grabImage() = 0;

      /// Get the camera matrix
      /**
       * \return Camera matrix
       */
      virtual cv::Mat K() = 0;

      /// Destructor
      virtual ~MonocularCamera() {}

    protected:
      /// Constructor
      MonocularCamera() {}
    };
  }  // namespace base
}  // namespace camera_plugins

#endif  // CAMERA_PLUGINS_BASE__MONOCULAR_HPP_