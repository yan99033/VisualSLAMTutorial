#include "vslam_nodes/utils.h"

#include <random>

namespace vslam_components {
  namespace vslam_nodes {
    namespace utils {
      double calculate_sim3_scale(const PointPairs& point_pairs, const cv::Mat& T_2_1, const int ransac_iters,
                                  size_t ransac_n) {
        // Rotation and translation
        cv::Matx33d R = T_2_1.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_2_1.rowRange(0, 3).colRange(3, 4);
        cv::Point3d t_pt = cv::Point3d(t);

        // Cap the data point to a max of half the size of point_pairs
        ransac_n = std::min(ransac_n, static_cast<size_t>(point_pairs.size() / 2));

        // random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, static_cast<int>(point_pairs.size() - 1));

        double smallest_err{std::numeric_limits<double>::max()};
        double best_scale{0.0};

        for (int i = 0; i < ransac_iters; i++) {
          // Randomly select n indices
          std::set<size_t> indices;
          while (indices.size() < ransac_n) {
            indices.insert(static_cast<size_t>(dis(gen)));
          }

          // calculate scale
          double numerator = 0;
          double denominator = 0;
          for (const auto& idx : indices) {
            const auto mp1 = point_pairs.at(idx).first;
            const auto mp2 = point_pairs.at(idx).second;

            const auto vec1 = R * mp1;
            const auto vec2 = mp2 - t_pt;

            numerator += vec1.dot(vec2);
            denominator += vec1.dot(vec1);
          }
          const double scale = [&] {
            if (denominator == 0.0) {
              return 0.0;
            } else {
              return numerator / denominator;
            }
          }();

          // calculate the fitting error
          double fitting_error{0.0};
          for (const auto& [mp1, mp2] : point_pairs) {
            fitting_error += cv::norm(R * mp1 + t_pt - mp2);
          }

          // if the error smaller than the smallest error
          if (fitting_error < smallest_err && scale != 0) {
            best_scale = scale;
            smallest_err = fitting_error;
          }
        }

        return best_scale;
      }

    }  // namespace utils
  }    // namespace vslam_nodes
}  // namespace vslam_components