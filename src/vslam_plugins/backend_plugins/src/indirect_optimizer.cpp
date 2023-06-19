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

#include "vslam_backend_plugins/indirect_optimizer.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "vslam_backend_plugins/utils.hpp"
#include "vslam_utils/converter.hpp"

namespace vslam_backend_plugins {
  IndirectOptimizer::~IndirectOptimizer() {
    exit_thread_ = true;

    if (local_ba_thread_.joinable()) {
      local_ba_condition_.notify_one();
      local_ba_thread_.join();
    }
    std::cerr << "Terminated IndirectOptimizer" << std::endl;
  }

  void IndirectOptimizer::initialize() { local_ba_thread_ = std::thread(&IndirectOptimizer::localBALoop, this); }

  void IndirectOptimizer::addKeyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->isKeyframe());

    {
      std::lock_guard<std::mutex> kf_lck(keyframe_mutex_);
      keyframes_[keyframe->id()] = keyframe;
      current_keyframe_ = keyframe;
    }

    if (!run_local_ba_ && !loop_optimization_running_) {
      run_local_ba_ = true;
      std::unique_lock<std::mutex> lck(local_ba_mutex_);
      local_ba_condition_.notify_one();
    }
  }

  void IndirectOptimizer::removeKeyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr IndirectOptimizer::getKeyframe(const long unsigned int id) const {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(id) != keyframes_.end()) {
      return keyframes_.at(id);
    } else {
      return nullptr;
    }
  }

  void IndirectOptimizer::localBALoop() {
    while (!exit_thread_) {
      {
        std::unique_lock<std::mutex> lck(local_ba_mutex_);
        local_ba_condition_.wait(lck, [this] { return exit_thread_ || run_local_ba_; });
      }

      if (exit_thread_) {
        break;
      }

      auto [core_keyframes, core_mappoints] = getCoreKeyframesMappoints();
      if (core_keyframes.size() < num_core_kfs_) {
        std::this_thread::yield();
        continue;
      }

      runBundleAdjustmentImpl(core_keyframes, core_mappoints);

      if (!cleaning_stale_keyframes_mappoints_ && !loop_optimization_running_) {
        cleaning_stale_keyframes_mappoints_ = true;
        cleanUpStaleKeyframesMappoints((*core_keyframes.begin())->id(), (*core_keyframes.rbegin())->id());
        cleaning_stale_keyframes_mappoints_ = false;
      }

      run_local_ba_ = false;
    }
  }

  std::pair<IndirectOptimizer::CoreKfsSet, IndirectOptimizer::CoreMpsSet>
  IndirectOptimizer::getCoreKeyframesMappoints() {
    assert(num_core_kfs_ > 0);

    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    CoreKfsSet core_keyframes;
    CoreMpsSet core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    while (kfs_it != keyframes_.rend() && core_keyframes.size() < num_core_kfs_) {
      auto kf = kfs_it->second;

      if (!kf.get() || kf->isBad()) {
        kfs_it++;
        continue;
      }

      core_keyframes.insert(kf.get());

      for (auto pt : kf->points()) {
        if (pt->hasMappoint() && pt->mappoint()->projections().size() > 1) {
          core_mappoints.insert(pt->mappoint().get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void IndirectOptimizer::cleanUpStaleKeyframesMappoints(const long unsigned int min_kf_id,
                                                         const long unsigned int max_kf_id) {
    std::vector<vslam_datastructure::Frame::SharedPtr> keyframes_to_remove;

    vslam_datastructure::Frame::SharedPtr prev_kf_ptr{nullptr};
    std::optional<double> prev_rel_translation;
    for (auto& [_, kf_ptr] : keyframes_) {
      if (!kf_ptr.get() || kf_ptr->isBad() || kf_ptr->id() < min_kf_id) {
        continue;
      }

      // If the keyframe is being optimized or used for camera tracking or larger than the max id, stop processing the
      // rest
      if (kf_ptr->id() > max_kf_id || kf_ptr->active_tracking_state || kf_ptr->active_ba_state) {
        break;
      }

      std::set<vslam_datastructure::Frame*> projected_keyframes;

      for (const auto& pt : kf_ptr->points()) {
        if (pt->hasMappoint() && pt->isMappointHost()) {
          // if there are less than two projections, remove the map point
          if (pt->mappoint()->projections().size() < 2) {
            pt->deleteMappoint();
            continue;
          }

          for (const auto other_pt : pt->mappoint()->projections()) {
            assert(other_pt->frame());

            if (other_pt->frame()->isBad()) {
              continue;
            }

            projected_keyframes.insert(other_pt->frame());
          }
        }
      }

      // If the map points weren't projected on more than two frames, the keyframe is an outlier keyframe
      if (projected_keyframes.size() < 2 && (!kf_ptr->active_tracking_state && !kf_ptr->active_ba_state)) {
        std::cerr << "setting kf bad: " << kf_ptr->id() << std::endl;
        kf_ptr->setBad();

        continue;
      }

      if (!prev_kf_ptr) {
        prev_kf_ptr = kf_ptr;
        continue;
      }

      if (!prev_rel_translation.has_value()) {
        cv::Mat T_prev_curr = prev_kf_ptr->T_f_w() * kf_ptr->T_w_f();
        prev_rel_translation = cv::norm(T_prev_curr.rowRange(0, 3).colRange(3, 4));
        prev_kf_ptr = kf_ptr;
        continue;
      }

      // Distance test
      // Normally if the PnP method fails, it will return a big pose jump or has a large rotation
      cv::Mat T_prev_curr = prev_kf_ptr->T_f_w() * kf_ptr->T_w_f();
      const double rel_translation = cv::norm(T_prev_curr.rowRange(0, 3).colRange(3, 4));
      const cv::Mat R_prev_curr = T_prev_curr.rowRange(0, 3).colRange(0, 3);
      const double rel_rotation = vslam_utils::conversions::rotationMatrixToRotationAngle(R_prev_curr);

      if ((rel_translation > outlier_rel_translation_scale_ * prev_rel_translation.value()
           || rel_rotation > max_outlier_rel_rotation_rad_)
          && (!kf_ptr->active_tracking_state && !kf_ptr->active_ba_state)) {
        kf_ptr->setBad();
        continue;
      }

      prev_kf_ptr = kf_ptr;
      prev_rel_translation = rel_translation;
    }
  }

  void IndirectOptimizer::addLoopConstraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                            const cv::Mat& T_1_2, const double sim3_scale) {
    if (loop_optimization_running_) {
      return;
    }

    // Check if the keyframes exist
    {
      std::lock_guard<std::mutex> lck(keyframe_mutex_);
      if ((keyframes_.find(kf_id_1) == keyframes_.end()) || (keyframes_.find(kf_id_2) == keyframes_.end())) {
        return;
      }

      // The loop constraint cannot be established if one of the keyframes is bad
      if (keyframes_.at(kf_id_1)->isBad() || keyframes_.at(kf_id_2)->isBad()) {
        return;
      }
    }

    loop_optimization_running_ = true;

    runPoseGraphOptimizationImpl(kf_id_1, kf_id_2, T_1_2, sim3_scale, keyframes_);

    if (!cleaning_stale_keyframes_mappoints_ && !run_local_ba_) {
      cleaning_stale_keyframes_mappoints_ = true;
      cleanUpStaleKeyframesMappoints();
      cleaning_stale_keyframes_mappoints_ = false;
    }

    loop_optimization_running_ = false;
  }

  std::vector<vslam_msgs::msg::Frame> IndirectOptimizer::getAllKeyframeMsgs() const {
    std::vector<vslam_msgs::msg::Frame> keyframe_msgs;
    for (const auto& [_, kf] : keyframes_) {
      if (kf->isBad()) {
        continue;
      }

      vslam_msgs::msg::Frame keyframe_msg;
      kf->toMsg(&keyframe_msg, true);
      keyframe_msgs.push_back(keyframe_msg);
    }

    return keyframe_msgs;
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::IndirectOptimizer, vslam_backend::base::Backend)