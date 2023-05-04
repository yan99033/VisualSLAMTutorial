// TODO: Licence

#ifndef VSLAM_NODES__UTILS_HPP_
#define VSLAM_NODES__UTILS_HPP_

#include <opencv2/opencv.hpp>

namespace vslam_components {
  namespace vslam_nodes {
    using PointPairs = std::vector<std::pair<cv::Point3d, cv::Point3d>>;
    namespace utils {
      double calculate_sim3_scale(const PointPairs& mappoint_pairs, const cv::Mat& T_2_1, const int ransac_iters = 100,
                                  size_t ransac_n = 10);

    }  // namespace utils
  }    // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__UTILS_HPP_