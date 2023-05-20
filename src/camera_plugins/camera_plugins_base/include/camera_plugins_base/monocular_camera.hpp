#ifndef CAMERA_PLUGINS_BASE__MONOCULAR_HPP_
#define CAMERA_PLUGINS_BASE__MONOCULAR_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace camera_plugins_base {
  class MonocularCamera {
  public:
    // Every camera has unique characteristics (e.g., focal lengths, principal point and distortion coefficient)
    // as well as a unique camera ID
    virtual void initialize(const std::string& params) = 0;

    virtual cv::Mat grab_image() = 0;

    virtual cv::Mat K() = 0;

    virtual ~MonocularCamera() {}

  protected:
    MonocularCamera() {}
  };
}  // namespace camera_plugins_base

#endif  // CAMERA_PLUGINS_BASE__MONOCULAR_HPP_