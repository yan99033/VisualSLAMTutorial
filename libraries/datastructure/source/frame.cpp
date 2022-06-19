#include "datastructure/frame.h"

namespace vslam_libs {

  namespace datastructure {
    Frame::Frame(const cv::Mat& image) : image(image) {}

  }  // namespace datastructure
}  // namespace vslam_libs