/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vslam_backend_plugins/utils.hpp"

#include "vslam_datastructure/point.hpp"

namespace vslam_backend_plugins {
  namespace utils {
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::SE3Quat(R, t);
    }

    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::Sim3(R, t, scale);
    }

    std::unordered_set<const vslam_datastructure::Frame*> getFrameMappointProjectedFrames(
        vslam_datastructure::Frame* const frame, const bool use_host_mps, const size_t min_projections,
        const size_t top_k_projections) {
      if (!frame || frame->isBad()) {
        return std::unordered_set<const vslam_datastructure::Frame*>();
      }

      std::map<vslam_datastructure::Frame*, size_t> projection_counts;

      for (const auto& pt : frame->points()) {
        if (!pt->hasMappoint() || (!pt->isMappointHost() && use_host_mps)) {
          continue;
        }

        for (const auto& other_pt_weakptr : pt->mappoint()->projections()) {
          if (auto other_pt = other_pt_weakptr.lock()) {
            if (auto other_pt_frame = other_pt->frame()) {
              if (other_pt_frame->isBad() || other_pt_frame.get() == frame) {
                continue;
              }

              if (projection_counts.find(other_pt_frame.get()) == projection_counts.end()) {
                projection_counts[other_pt_frame.get()] = 1;
              } else {
                projection_counts[other_pt_frame.get()] += 1;
              }
            }
          }
        }
      }

      // Remove the frames that the projection count lower than the threshold
      for (auto it = projection_counts.begin(); it != projection_counts.end();) {
        if (it->second < min_projections) {
          it = projection_counts.erase(it);
        } else {
          ++it;
        }
      }

      // Weigh the projection count by the keyframe distance
      std::vector<std::pair<double, vslam_datastructure::Frame*>> weight_projections;
      for (const auto& [other_frame, count] : projection_counts) {
        const double weight = static_cast<double>(count)
                              / abs(static_cast<double>(frame->id()) - static_cast<double>(other_frame->id()));
        weight_projections.emplace_back(weight, other_frame);
      }

      // Sort and take the top k
      std::sort(weight_projections.begin(), weight_projections.end(),
                [](const std::pair<double, vslam_datastructure::Frame*>& lhs,
                   const std::pair<double, vslam_datastructure::Frame*>& rhs) { return lhs.first > rhs.first; });
      if (weight_projections.size() > top_k_projections) {
        weight_projections.resize(top_k_projections);
      }

      std::unordered_set<const vslam_datastructure::Frame*> projected_frames;
      for (auto [_, frame] : weight_projections) {
        projected_frames.insert(frame);
      }

      return projected_frames;
    }

  }  // namespace utils
}  // namespace vslam_backend_plugins