#include "vslam_utils/converter.hpp"

namespace vslam_utils {
  namespace conversions {
    cv::Mat toTransformationMatrix(const cv::Mat& R, const cv::Mat& t) {
      // Create an identity matrix
      cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

      // Copy data
      R.copyTo(T(cv::Rect(0, 0, 3, 3)));
      t.copyTo(T(cv::Rect(3, 0, 1, 3)));

      return T;
    }
  }  // namespace conversions
}  // namespace vslam_utils