#include "camera_plugins_base/monocular_camera.hpp"
#include "vslam_utils/undistorter.hpp"

namespace monocular_camera_plugins {
  class MonocularCamera : public camera_plugins::base::MonocularCamera {
  public:
    ~MonocularCamera();

    // Read the parameter files
    void initialize(const std::string& params_file) override;

    cv::Mat grab_image() override;

    cv::Mat K() override { return K_.clone(); }

  protected:
    void open_camera(int camera_id);

    cv::FileStorage params_fs_;

    int image_height_{-1};
    int image_width_{-1};
    int camera_id_{-1};

    // Undistorter
    std::unique_ptr<vslam_utils::Undistorter> undistorter_{nullptr};

    cv::Mat K_;

  private:
    cv::VideoCapture video_capture_;

    // Keeping the last image allows us to use it if VideoCapture fails to retrieve an image
    cv::Mat last_image_;
  };
}  // namespace monocular_camera_plugins