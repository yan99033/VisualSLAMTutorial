#include "datastructure/frame.h"

namespace vslam_libs {

  namespace datastructure {
    Frame::Frame(const cv::Mat& image) : image(image) {}

    Frame::Frame(const cv::Mat& image, const cv::Mat& cam_mat) : image(image), cam_mat(cam_mat) {}

  }  // namespace datastructure
}  // namespace vslam_libs