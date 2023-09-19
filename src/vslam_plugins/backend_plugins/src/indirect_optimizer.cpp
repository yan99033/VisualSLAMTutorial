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

  void IndirectOptimizer::initialize(vslam_datastructure::Map* map) {
    map_ = map;
    local_ba_thread_ = std::thread(&IndirectOptimizer::localBALoop, this);
  }

  void IndirectOptimizer::runLocalBA() {
    if (!run_local_ba_ && !loop_optimization_running_) {
      run_local_ba_ = true;
      std::unique_lock<std::mutex> lck(local_ba_mutex_);
      local_ba_condition_.notify_one();
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

      auto [core_keyframes, core_mappoints] = map_->getCoreKeyframesMappoints(num_core_kfs_);
      if (core_keyframes.size() < num_core_kfs_) {
        std::this_thread::yield();
        continue;
      }

      runBundleAdjustmentImpl(core_keyframes, core_mappoints);

      map_->cleanUpStaleKeyframesMappoints((*core_keyframes.begin())->id(), (*core_keyframes.rbegin())->id());

      run_local_ba_ = false;
    }
  }

  void IndirectOptimizer::addLoopConstraint(const vslam_datastructure::Sim3Constraint& sim3_constraint) {
    if (loop_optimization_running_) {
      return;
    }

    // Check if the keyframes are good
    if (!sim3_constraint.keyframe1 || !sim3_constraint.keyframe2 || sim3_constraint.keyframe1->isBad()
        || sim3_constraint.keyframe2->isBad()) {
      return;
    }

    loop_optimization_running_ = true;

    auto keyframes = map_->keyframes();
    runPoseGraphOptimizationImpl(sim3_constraint, keyframes);

    map_->cleanUpStaleKeyframesMappoints();

    loop_optimization_running_ = false;
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::IndirectOptimizer, vslam_backend::base::Backend)