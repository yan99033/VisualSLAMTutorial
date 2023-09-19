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

    utils::transferOptimizedSparseBAResults(results, huber_kernel_delta_sq_);

    for (auto& core_kf : core_keyframes) {
      core_kf->active_ba_state = false;
    }
  }

  void Optimizer::runPoseGraphOptimizationImpl(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                               const cv::Mat& T_1_2, const double sim3_scale,
                                               std::list<vslam_datastructure::Frame::SharedPtr>& keyframes) {
    if (!map_) {
      std::cerr << "Map is not set. Not running pose graph optimizartion." << std::endl;
      return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    utils::setupPoseGraphOptimizer(optimizer);

    // Create vertices
    unsigned long int vertex_edge_id{0};
    std::unordered_map<long unsigned int, g2o::VertexSim3Expmap*> kf_vertices;
    for (const auto& kf : keyframes) {
      if (kf->isBad()) {
        continue;
      }

      g2o::VertexSim3Expmap* v_sim3 = new g2o::VertexSim3Expmap();
      v_sim3->setId(vertex_edge_id++);
      v_sim3->setEstimate(utils::cvMatToSim3(kf->T_f_w(), 1.0));
      v_sim3->setFixed(kf->active_tracking_state);
      v_sim3->setMarginalized(false);
      optimizer.addVertex(v_sim3);

      kf_vertices[kf->id()] = v_sim3;

      if (kf->active_tracking_state) {
        break;
      }
    }

    // If the loop edge cannot be established
    if (kf_vertices.find(kf_id_1) == kf_vertices.end() || kf_vertices.find(kf_id_2) == kf_vertices.end()) {
      return;
    }

    // Create edges
    for (const auto& kf : keyframes) {
      if (kf->isBad() || kf_vertices.find(kf->id()) == kf_vertices.end()) {
        continue;
      }

      auto v_sim3_this = kf_vertices.at(kf->id());

      if (kf->nearby_keyframes.empty()) {
        kf->nearby_keyframes = utils::getFrameMappointProjectedFrames(kf);
      }

      if (kf->nearby_keyframes.size() < 3) {
        kf->setBad();
        continue;
      }

      bool need_recalculating_nearby_keyframes{false};
      for (const auto& other_kf : kf->nearby_keyframes) {
        if (kf_vertices.find(other_kf->id()) == kf_vertices.end()) {
          continue;
        }

        if (other_kf->isBad()) {
          need_recalculating_nearby_keyframes = true;
          continue;
        }

        cv::Mat T_this_other = kf->T_f_w() * other_kf->T_w_f();

        auto v_sim3_other = kf_vertices.at(other_kf->id());

        g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
        e_sim3->setId(vertex_edge_id++);
        e_sim3->setVertex(0, v_sim3_this);
        e_sim3->setVertex(1, v_sim3_other);
        e_sim3->setMeasurement(utils::cvMatToSim3(T_this_other, 1.0).inverse());
        e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

        optimizer.addEdge(e_sim3);
      }

      // Recalculate the nearby keyframes if there are new bad keyframes
      if (need_recalculating_nearby_keyframes) {
        kf->nearby_keyframes.clear();
      }

      for (auto it = kf->loop_keyframes.begin(); it != kf->loop_keyframes.end();) {
        if (kf_vertices.find((*it)->id()) == kf_vertices.end()) {
          ++it;
          continue;
        }

        if ((*it)->isBad()) {
          it = kf->loop_keyframes.erase(it);
          ++it;
          continue;
        }

        cv::Mat T_this_other = kf->T_f_w() * (*it)->T_w_f();

        auto v_sim3_other = kf_vertices.at((*it)->id());

        g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
        e_sim3->setId(vertex_edge_id++);
        e_sim3->setVertex(0, v_sim3_this);
        e_sim3->setVertex(1, v_sim3_other);
        e_sim3->setMeasurement(utils::cvMatToSim3(T_this_other, 1.0).inverse());
        e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

        optimizer.addEdge(e_sim3);

        ++it;
      }

      if (kf->active_tracking_state) {
        break;
      }
    }

    auto v_sim3_1 = kf_vertices.at(kf_id_1);
    auto v_sim3_2 = kf_vertices.at(kf_id_2);
    g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
    e_sim3->setId(vertex_edge_id++);
    e_sim3->setVertex(0, v_sim3_1);
    e_sim3->setVertex(1, v_sim3_2);
    e_sim3->setMeasurement(utils::cvMatToSim3(T_1_2, sim3_scale).inverse());
    e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

    optimizer.addEdge(e_sim3);

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

    // Recalculate SE(3) poses and map points in their host keyframe
    for (const auto& [kf_id, kf_vertex] : kf_vertices) {
      auto kf = map_->getKeyframe(kf_id);

      // If the keyframe is being used or optimized
      if (!kf || kf->active_tracking_state || kf->active_ba_state || kf->isBad()) {
        continue;
      }

      // calculate the pose and scale
      const auto g2o_S_f_w = kf_vertex->estimate();
      const cv::Mat temp_T_f_w = vslam_utils::conversions::eigenRotationTranslationToCvMat(
          g2o_S_f_w.rotation().toRotationMatrix(), g2o_S_f_w.translation());
      const double scale = g2o_S_f_w.scale();

      cv::Mat T_f_w = temp_T_f_w.clone();
      T_f_w.rowRange(0, 3).colRange(3, 4) /= scale;
      cv::Mat S_f_w = temp_T_f_w.clone();
      S_f_w.rowRange(0, 3).colRange(0, 3) *= scale;

      kf->updateSim3PoseAndMps(S_f_w, T_f_w);
    }

    // Add the loop keyframe
    auto kf_1 = map_->getKeyframe(kf_id_1);
    auto kf_2 = map_->getKeyframe(kf_id_2);
    if (kf_1 && kf_2) {
      kf_1->loop_keyframes.insert(kf_2);
      kf_2->loop_keyframes.insert(kf_1);
    }
  }

}  // namespace vslam_backend_plugins