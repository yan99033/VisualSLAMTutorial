#ifndef MONOCULAR_CAMERA_PLUGINS__MONOCULAR_CAMERA_HPP_
#define MONOCULAR_CAMERA_PLUGINS__MONOCULAR_CAMERA_HPP_

#include "camera_plugins_base/monocular_camera.hpp"
#include "vslam_utils/undistorter.hpp"

namespace monocular_camera_plugins {
  namespace abstract {
    class MonocularCamera : public virtual camera_plugins::base::MonocularCamera {
    protected:
      /// Open the USB camera
      /**
       * \param[in] camera_id the USB camera ID
       */
      virtual void openCamera(int camera_id) = 0;
    };
  }  // namespace abstract

  class MonocularCamera : public virtual monocular_camera_plugins::abstract::MonocularCamera {
  public:
    ~MonocularCamera();

    /// Monocular camera initializer
    void initialize(const std::string& params_file) override;

    /// Grab the next image from the camera
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
    /// Open the USB camera
    /**
     * \param[in] camera_id the USB camera ID
     */
    void openCamera(int camera_id) override;

    /// File storage object for keeping the parameters
    cv::FileStorage params_fs_;

    /// Image height
    int image_height_{-1};

    /// Image width
    int image_width_{-1};

    /// Camera id
    int camera_id_{-1};

    /// Undistorter
    std::unique_ptr<vslam_utils::camera::Undistorter> undistorter_{nullptr};

    /// Camera matrix
    cv::Mat K_;

  private:
    /// Capture video from cameras
    /**
     * \sa https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
     */
    cv::VideoCapture video_capture_;

    // Keeping the last image allows us to use it if VideoCapture fails to retrieve an image
    cv::Mat last_image_;
  };
}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS__MONOCULAR_CAMERA_HPP_