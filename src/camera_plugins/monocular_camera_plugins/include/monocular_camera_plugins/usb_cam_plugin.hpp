#ifndef MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_

#include "camera_plugins_base/monocular_camera.hpp"

namespace monocular_camera_plugins {
  class UsbCamera : public camera_plugins_base::MonocularCamera {
  public:
    ~UsbCamera();

    void initialize(const std::string& params) override;

    cv::Mat grab_image() override;

    cv::Mat K() override { return K_.clone(); }

  private:
    int image_height_{-1};
    int image_width_{-1};
    int camera_id_{-1};

    cv::Mat K_;

    cv::VideoCapture video_capture_;

    // Keeping the last image allows us to use it if VideoCapture fails to retrieve an image
    cv::Mat last_image_;
  };
}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS__GENERIC_USB_CAM_PLUGIN_HPP_