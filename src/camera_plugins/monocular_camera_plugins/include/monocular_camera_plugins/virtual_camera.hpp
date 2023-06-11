#ifndef MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_

#include "camera_plugins_base/monocular_camera.hpp"
#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {
  namespace abstract {
    class VirtualCamera : public virtual camera_plugins::base::MonocularCamera {
    protected:
      /// Get the filenames in a folder
      /**
       * \param folder[in] path to a folder containing the images
       * \param ext[in] image extension (e.g., ".png" or ".jpg")
       */
      virtual void loadFromFolder(const std::string& folder, const std::string& ext) = 0;
    };
  }  // namespace abstract

  class VirtualCamera : public virtual abstract::VirtualCamera, public MonocularCamera {
  public:
    /// Virtual camera initializer
    void initialize(const std::string& params_file) override;

    /// Grab the next image from the folder
    /**
     * \return An undistorted image
     */
    cv::Mat grabImage() override;

    /// Get the camera matrix
    /**
     * \return Camera matrix
     */
    cv::Mat K() override { return K_.clone(); }

  protected:
    /// Get the filenames in a folder
    /**
     * \param folder[in] path to a folder containing the images
     * \param ext[in] image extension (e.g., ".png" or ".jpg")
     */
    void loadFromFolder(const std::string& folder, const std::string& ext = ".png") override;

  private:
    /// The current index in the files_ vector
    size_t i_{0};

    /// filenames in a sequential order
    std::vector<std::string> files_;
  };

}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_