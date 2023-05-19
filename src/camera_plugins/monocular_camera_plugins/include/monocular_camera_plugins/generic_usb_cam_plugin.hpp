#ifndef MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_

#include "camera_plugins_base/monocular_camera.hpp"

namespace monocular_camera_plugins {
  class GenericUsbCamera : public camera_plugins_base::MonocularCamera {
  public:
    ~GenericUsbCamera();

    virtual void initialize(const std::string& params) override;

    virtual cv::Mat grab_image() override;

  private:
    int image_height_{-1};
    int image_width_{-1};
    int camera_id_{-1};

    cv::VideoCapture video_capture_;
  };
}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_