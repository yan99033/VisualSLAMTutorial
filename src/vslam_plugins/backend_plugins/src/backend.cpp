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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#if defined G2O_HAVE_CHOLMOD
#  include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#else
#  include "g2o/solvers/eigen/linear_solver_eigen.h"
#endif

#include "vslam_backend_plugins/utils.hpp"
#include "vslam_utils/converter.hpp"

namespace vslam_backend_plugins {
  Optimizer::~Optimizer() { std::cerr << "Terminated Optimizer" << std::endl; }

  void Optimizer::runBundleAdjustmentImpl(vslam_datastructure::CoreKfsSet& core_keyframes,
                                          vslam_datastructure::CoreMpsSet& core_mappoints) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
#ifdef G2O_HAVE_CHOLMOD
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver
        = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
#else
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver
        = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
#endif
    g2o::OptimizationAlgorithmLevenberg* solver
        = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
    optimizer.setAlgorithm(solver);

    for (auto core_kf : core_keyframes) {
      core_kf->active_ba_state = true;
    }

    // Create vertices and edges
    std::map<g2o::VertexPointXYZ*, vslam_datastructure::MapPoint*> core_mp_vertices;
    std::map<g2o::VertexSE3Expmap*, vslam_datastructure::Frame*> core_kf_vertices;
    std::list<g2o::EdgeSE3ProjectXYZ*> all_edges;
    std::map<vslam_datastructure::Frame*, g2o::VertexSE3Expmap*> existing_kf_vertices;
    std::map<g2o::EdgeSE3ProjectXYZ*, std::pair<vslam_datastructure::MapPoint*, vslam_datastructure::Point*>>
        edge_projections;
    unsigned long int vertex_edge_id{0};
    for (auto mp : core_mappoints) {
      if (mp == nullptr || mp->isOutlier()) {
        continue;
      }

      // Check if we have at least two valid projections
      int num_valid_projections{0};
      for (auto pt : mp->projections()) {
        if (pt == nullptr || !pt->hasFrame() || pt->frame()->isBad()) {
          continue;
        }
        num_valid_projections++;
      }
      if (num_valid_projections < 2) {
        continue;
      }

      g2o::VertexPointXYZ* mp_vertex = new g2o::VertexPointXYZ();
      core_mp_vertices[mp_vertex] = mp;
      mp_vertex->setEstimate(vslam_utils::conversions::cvPoint3dToEigenVector3d(mp->pos()));
      mp_vertex->setId(vertex_edge_id++);
      mp_vertex->setMarginalized(true);
      optimizer.addVertex(mp_vertex);

      // Projections
      for (auto pt : mp->projections()) {
        if (pt == nullptr || !pt->hasFrame() || pt->frame()->isBad()) {
          continue;
        }

        // Keyframe vertex
        g2o::VertexSE3Expmap* kf_vertex;
        if (existing_kf_vertices.find(pt->frame()) != existing_kf_vertices.end()) {
          kf_vertex = existing_kf_vertices[pt->frame()];
        } else {
          kf_vertex = new g2o::VertexSE3Expmap();
          existing_kf_vertices[pt->frame()] = kf_vertex;
          kf_vertex->setEstimate(utils::cvMatToSE3Quat(pt->frame()->T_f_w()));
          kf_vertex->setId(vertex_edge_id++);
          if (core_keyframes.find(pt->frame()) != core_keyframes.end()) {
            // Core keyframes
            core_kf_vertices[kf_vertex] = pt->frame();
            kf_vertex->setFixed(pt->frame()->active_tracking_state);
          } else {
            // Non-core keyframes
            kf_vertex->setFixed(true);
          }
          optimizer.addVertex(kf_vertex);
        }

        // Projection
        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        e->setId(vertex_edge_id++);
        e->setVertex(0, mp_vertex);
        e->setVertex(1, kf_vertex);
        e->setMeasurement(vslam_utils::conversions::cvPoint2fToEigenVector2d(pt->keypoint.pt));
        e->setInformation(Eigen::Matrix2d::Identity());  // TODO: incorporate uncertainty

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(huber_kernel_delta_);
        e->setRobustKernel(rk);

        edge_projections[e] = {mp, pt};

        const cv::Mat K = pt->frame()->K();

        e->fx = K.at<double>(0, 0);
        e->fy = K.at<double>(1, 1);
        e->cx = K.at<double>(0, 2);
        e->cy = K.at<double>(1, 2);

        optimizer.addEdge(e);
        all_edges.push_back(e);
      }
    }

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

    // Update keyframes
    for (auto [kf_vertex, kf_p] : core_kf_vertices) {
      auto T_f_w = kf_vertex->estimate();
      cv::Mat cv_T_f_w = vslam_utils::conversions::eigenRotationTranslationToCvMat(T_f_w.rotation().toRotationMatrix(),
                                                                                   T_f_w.translation());

      if (kf_p) {
        kf_p->setPose(cv_T_f_w);
      }
    }

    for (auto e : all_edges) {
      if (!e->isDepthPositive() || e->chi2() > huber_kernel_delta_sq_) {
        auto [mp_p, pt_p] = edge_projections[e];

        if (mp_p && pt_p) {
          mp_p->removeProjection(pt_p);
        }
      }
    }

    // Update map points
    int inlier_mappoints{0};
    for (auto [mp_vertex, mp_p] : core_mp_vertices) {
      auto mp = mp_vertex->estimate();
      if (mp_p && mp.hasNaN()) {
        mp_p->setOutlier();
        continue;
      }

      if (mp_p && !mp_p->isOutlier()) {
        auto pt_3d = vslam_utils::conversions::eigenVector3dToCvPoint3d(mp);

        mp_p->setPos(pt_3d);
        inlier_mappoints++;
      }
    }

    for (auto core_kf : core_keyframes) {
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
    optimizer.setVerbose(false);
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>> BlockSolverType;
#ifdef G2O_HAVE_CHOLMOD
    typedef g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType> LinearSolverType;
#else
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
#endif

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver);

    // Create vertices
    unsigned long int vertex_edge_id{0};
    std::map<long unsigned int, g2o::VertexSim3Expmap*> kf_vertices;
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
        kf->nearby_keyframes = utils::getFrameMappointProjectedFrames(kf.get());
      }

      if (kf->nearby_keyframes.size() < 3) {
        kf->setBad();
        continue;
      }

      bool need_recalculating_nearby_keyframes{false};
      for (const auto other_kf : kf->nearby_keyframes) {
        if (kf_vertices.find(other_kf->id()) == kf_vertices.end()) {
          continue;
        }

        if (other_kf->isBad()) {
          need_recalculating_nearby_keyframes = true;
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
    for (const auto [kf_id, kf_vertex] : kf_vertices) {
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
      kf_1->loop_keyframes.insert(kf_2.get());
      kf_2->loop_keyframes.insert(kf_1.get());
    }
  }

}  // namespace vslam_backend_plugins