/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vslam_nodes/utils.hpp"

#include <limits>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <random>
#include <tf2_eigen/tf2_eigen.hpp>

namespace vslam_components {
  namespace vslam_nodes {
    namespace utils {
      double calculateSim3Scale(const PointPairs& mappoint_pairs, const cv::Mat& T_2_1, const int ransac_iters,
                                size_t ransac_n) {
        // Rotation and translation
        cv::Matx33d R = T_2_1.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_2_1.rowRange(0, 3).colRange(3, 4);
        cv::Point3d t_pt = cv::Point3d(t);

        // Cap the data point to a max of half the size of mappoint_pairs
        ransac_n = std::min(ransac_n, static_cast<size_t>(mappoint_pairs.size() / 2));

        // random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, static_cast<int>(mappoint_pairs.size() - 1));

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
            const auto mp1 = mappoint_pairs.at(idx).first;
            const auto mp2 = mappoint_pairs.at(idx).second;

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
          const double fitting_error = [&] {
            if (scale <= 0) {
              return std::numeric_limits<double>::max();
            } else {
              double total_error{0.0};
              for (const auto& [mp1, mp2] : mappoint_pairs) {
                const auto mp2_prime = scale * R * mp1 + t_pt;
                const double err = cv::norm(mp2_prime - mp2);
                total_error += err;
              }
              return total_error;
            }
          }();

          // if the error smaller than the smallest error
          if (fitting_error < smallest_err && scale > 0) {
            best_scale = scale;
            smallest_err = fitting_error;
          }
        }

        // Check the percentage of outlier points
        double num_outliers{0};
        for (const auto& [mp1, mp2] : mappoint_pairs) {
          const auto mp2_prime = best_scale * R * mp1 + t_pt;
          const double err = cv::norm(mp2_prime - mp2);

          // outlier if the magnitude of the error is greater than 15 % of the length
          if (err > cv::norm(mp2) * 0.15) {
            num_outliers += 1.0;
          }
        }

        if (num_outliers / static_cast<double>(mappoint_pairs.size()) > 0.3) {
          std::cout << "num outliers: " << num_outliers << " / " << mappoint_pairs.size() << std::endl;

          return 0.0;
        }

        return best_scale;
      }
    }  // namespace utils
  }    // namespace vslam_nodes
}  // namespace vslam_components