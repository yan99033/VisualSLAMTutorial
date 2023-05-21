#ifndef MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_

#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {
  // For more information about the camera, see here
  // https://github.com/astar-ai/calicam
  class CaliCam : public MonocularCamera {
  public:
    void initialize(const std::string& params) override;
  };
}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_