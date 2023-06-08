#ifndef MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_

#include "camera_plugins_base/monocular_camera.hpp"
#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {
  namespace abstract {
    class VirtualCamera : public virtual camera_plugins::base::MonocularCamera {
    protected:
      virtual void load_from_folder(const std::string& folder, const std::string& ext) = 0;
    };
  }  // namespace abstract

  class VirtualCamera : public virtual abstract::VirtualCamera, public MonocularCamera {
  public:
    void initialize(const std::string& params_file) override;

    cv::Mat grab_image() override;

    cv::Mat K() override { return K_.clone(); }

  protected:
    void load_from_folder(const std::string& folder, const std::string& ext = ".png") override;

  private:
    size_t i_{0};

    std::vector<std::string> files_;  //!< Files in a folder
  };

}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_