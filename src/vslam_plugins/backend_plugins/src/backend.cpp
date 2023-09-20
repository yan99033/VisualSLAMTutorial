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

#include "vslam_backend_plugins/backend.hpp"

#include <g2o/core/sparse_optimizer.h>

#include "vslam_backend_plugins/utils.hpp"
#include "vslam_utils/converter.hpp"

namespace vslam_backend_plugins {
  Optimizer::~Optimizer() { std::cerr << "Terminated Optimizer" << std::endl; }

  void Optimizer::runBundleAdjustmentImpl(vslam_datastructure::CoreKfsSet& core_keyframes,
                                          vslam_datastructure::CoreMpsSet& core_mappoints) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    utils::setupSparseBAOptimizer(optimizer);

    for (auto& core_kf : core_keyframes) {
      core_kf->active_ba_state = true;
    }

    // Create vertices and edges
    types::SparseBAResults results;
    utils::constructSparseBAGraph(core_keyframes, core_mappoints, optimizer, huber_kernel_delta_, results);

    // optimize graph
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    const double init_errs = optimizer.activeChi2();
    optimizer.optimize(ba_iterations_);

    // If the errors after optimization is larger than before
    const double final_errs = optimizer.activeChi2();
    if (init_errs < final_errs) {
      return;
    }

    // Transfer optimized graph results back to SLAM
    utils::transferOptimizedSparseBAResults(results, huber_kernel_delta_sq_);

    for (auto& core_kf : core_keyframes) {
      core_kf->active_ba_state = false;
    }
  }

  void Optimizer::runPoseGraphOptimizationImpl(const vslam_datastructure::Sim3Constraint& sim3_constraint,
                                               std::list<vslam_datastructure::Frame::SharedPtr>& keyframes) {
    if (!map_) {
      std::cerr << "Map is not set. Not running pose graph optimizartion." << std::endl;
      return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    utils::setupPoseGraphOptimizer(optimizer);

    // Create vertices and edges
    types::poseGraphOptimizationResults results;
    utils::constructPoseGraph(keyframes, sim3_constraint, optimizer, results);

    // Optimize pose graph
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    const double init_errs = optimizer.activeChi2();
    optimizer.optimize(pgo_iterations_);

    // If the errors after optimization is larger than before
    const double final_errs = optimizer.activeChi2();
    if (init_errs < final_errs) {
      return;
    }

    // Transfer optimized graph results back to SLAM
    utils::transferOptimizedPoseGraphResults(results);

    // Add the loop keyframe
    sim3_constraint.keyframe1->loop_keyframes.insert(sim3_constraint.keyframe2);
    sim3_constraint.keyframe2->loop_keyframes.insert(sim3_constraint.keyframe1);
  }

}  // namespace vslam_backend_plugins