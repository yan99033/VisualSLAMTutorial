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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

#include "vslam_datastructure/point.hpp"
#include "vslam_utils/converter.hpp"

#if defined G2O_HAVE_CHOLMOD
#  include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#else
#  include <g2o/solvers/eigen/linear_solver_eigen.h>
#endif

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

    void setupSparseBAOptimizer(g2o::SparseOptimizer& optimizer) {
      // Setup g2o optimizer
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
    }

    void setupPoseGraphOptimizer(g2o::SparseOptimizer& optimizer) {
      // Setup g2o optimizer
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
    }

    void constructSparseBAGraph(const vslam_datastructure::CoreKfsSet& core_keyframes,
                                const vslam_datastructure::CoreMpsSet& core_mappoints, g2o::SparseOptimizer& optimizer,
                                const double huber_kernel_delta, types::SparseBAResults& results) {
      std::unordered_map<vslam_datastructure::Frame::SharedPtr, g2o::VertexSE3Expmap*> existing_kf_vertices;
      unsigned long int vertex_edge_id{0};
      for (auto& mp : core_mappoints) {
        if (mp->isOutlier()) {
          continue;
        }

        // Check if we have at least two valid projections
        vslam_datastructure::PointFramePairSet valid_projections;
        for (auto& pt_weakptr : mp->projections()) {
          if (auto pt = pt_weakptr.lock()) {
            if (auto frame = pt->frame()) {
              if (!frame->isBad()) {
                valid_projections.insert({pt, frame});
              }
            }
          }
        }
        if (valid_projections.size() < 2) {
          continue;
        }

        g2o::VertexPointXYZ* mp_vertex = new g2o::VertexPointXYZ();
        results.mappoint_vertices[mp] = mp_vertex;
        mp_vertex->setEstimate(vslam_utils::conversions::cvPoint3dToEigenVector3d(mp->pos()));
        mp_vertex->setId(vertex_edge_id++);
        mp_vertex->setMarginalized(true);
        optimizer.addVertex(mp_vertex);

        // Projections
        for (auto& [pt, frame] : valid_projections) {
          // Keyframe vertex
          g2o::VertexSE3Expmap* kf_vertex;
          if (existing_kf_vertices.find(frame) != existing_kf_vertices.end()) {
            kf_vertex = existing_kf_vertices[frame];
          } else {
            kf_vertex = new g2o::VertexSE3Expmap();
            existing_kf_vertices[frame] = kf_vertex;
            kf_vertex->setEstimate(utils::cvMatToSE3Quat(frame->T_f_w()));
            kf_vertex->setId(vertex_edge_id++);
            if (core_keyframes.find(frame) != core_keyframes.end()) {
              // Core keyframes
              results.keyframe_vertices[frame] = kf_vertex;
              kf_vertex->setFixed(frame->active_tracking_state);
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
          rk->setDelta(huber_kernel_delta);
          e->setRobustKernel(rk);

          const cv::Mat K = pt->frame()->K();

          e->fx = K.at<double>(0, 0);
          e->fy = K.at<double>(1, 1);
          e->cx = K.at<double>(0, 2);
          e->cy = K.at<double>(1, 2);

          optimizer.addEdge(e);
          results.edges[e] = {pt, mp};
        }
      }
    }

    void constructPoseGraph(const std::list<vslam_datastructure::Frame::SharedPtr>& keyframes,
                            const vslam_datastructure::Sim3Constraint& sim3_constraint, g2o::SparseOptimizer& optimizer,
                            types::poseGraphOptimizationResults& results) {
      // Create vertices
      unsigned long int vertex_edge_id{0};
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

        results[kf] = v_sim3;

        if (kf->active_tracking_state) {
          break;
        }
      }

      // If the loop edge cannot be established
      if (results.find(sim3_constraint.keyframe1) == results.end()
          || results.find(sim3_constraint.keyframe2) == results.end()) {
        return;
      }

      // Create edges
      for (const auto& kf : keyframes) {
        if (kf->isBad() || results.find(kf) == results.end()) {
          continue;
        }

        auto v_sim3_this = results.at(kf);

        if (kf->nearby_keyframes.empty()) {
          kf->nearby_keyframes = utils::getFrameMappointProjectedFrames(kf);
        }

        if (kf->nearby_keyframes.size() < 3) {
          kf->setBad();
          continue;
        }

        bool need_recalculating_nearby_keyframes{false};
        for (const auto& other_kf : kf->nearby_keyframes) {
          if (results.find(other_kf) == results.end()) {
            continue;
          }

          if (other_kf->isBad()) {
            need_recalculating_nearby_keyframes = true;
            continue;
          }

          cv::Mat T_this_other = kf->T_f_w() * other_kf->T_w_f();

          auto v_sim3_other = results.at(other_kf);

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
          if (results.find(*it) == results.end()) {
            ++it;
            continue;
          }

          if ((*it)->isBad()) {
            it = kf->loop_keyframes.erase(it);
            ++it;
            continue;
          }

          cv::Mat T_this_other = kf->T_f_w() * (*it)->T_w_f();

          auto v_sim3_other = results.at(*it);

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

      auto v_sim3_1 = results.at(sim3_constraint.keyframe1);
      auto v_sim3_2 = results.at(sim3_constraint.keyframe2);
      g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
      e_sim3->setId(vertex_edge_id++);
      e_sim3->setVertex(0, v_sim3_1);
      e_sim3->setVertex(1, v_sim3_2);
      e_sim3->setMeasurement(utils::cvMatToSim3(sim3_constraint.T_1_2, sim3_constraint.scale).inverse());
      e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

      optimizer.addEdge(e_sim3);
    }

    void transferOptimizedSparseBAResults(const types::SparseBAResults& results, const double huber_kernel_delta_sq) {
      // Update keyframes
      for (auto& [kf_p, kf_vertex] : results.keyframe_vertices) {
        assert(kf_vertex != nullptr);

        auto T_f_w = kf_vertex->estimate();
        cv::Mat cv_T_f_w = vslam_utils::conversions::eigenRotationTranslationToCvMat(
            T_f_w.rotation().toRotationMatrix(), T_f_w.translation());

        kf_p->setPose(cv_T_f_w);
      }

      // Check projections
      for (auto& [e, pt_mp_pair] : results.edges) {
        assert(e != nullptr);
        if (!e->isDepthPositive() || e->chi2() > huber_kernel_delta_sq) {
          auto& [pt_p, mp_p] = pt_mp_pair;

          mp_p->removeProjection(pt_p);
        }
      }

      // Update map points
      for (auto& [mp_p, mp_vertex] : results.mappoint_vertices) {
        assert(mp_vertex != nullptr);

        auto mp = mp_vertex->estimate();
        if (mp.hasNaN()) {
          mp_p->setOutlier();
          continue;
        }

        if (!mp_p->isOutlier()) {
          auto pt_3d = vslam_utils::conversions::eigenVector3dToCvPoint3d(mp);

          mp_p->setPos(pt_3d);
        }
      }
    }

    void transferOptimizedPoseGraphResults(const types::poseGraphOptimizationResults& results) {
      // Recalculate SE(3) poses and map points in their host keyframe
      for (const auto& [kf, kf_vertex] : results) {
        assert(kf_vertex != nullptr);

        // If the keyframe is being used or optimized
        if (kf->active_tracking_state || kf->active_ba_state || kf->isBad()) {
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
    }

    vslam_datastructure::FrameSet getFrameMappointProjectedFrames(const vslam_datastructure::Frame::SharedPtr frame,
                                                                  const bool use_host_mps, const size_t min_projections,
                                                                  const size_t top_k_projections) {
      if (!frame || frame->isBad()) {
        return vslam_datastructure::FrameSet();
      }

      std::unordered_map<vslam_datastructure::Frame::SharedPtr, size_t> projection_counts;

      for (const auto& pt : frame->points()) {
        if (!pt->hasMappoint() || (!pt->isMappointHost() && use_host_mps)) {
          continue;
        }

        for (const auto& other_pt_weakptr : pt->mappoint()->projections()) {
          if (auto other_pt = other_pt_weakptr.lock()) {
            if (auto other_pt_frame = other_pt->frame()) {
              if (other_pt_frame->isBad() || other_pt_frame->id() == frame->id()) {
                continue;
              }

              if (projection_counts.find(other_pt_frame) == projection_counts.end()) {
                projection_counts[other_pt_frame] = 1;
              } else {
                projection_counts[other_pt_frame] += 1;
              }
            }
          }
        }
      }

      // Weigh the projection count by the keyframe distance
      std::vector<std::pair<double, vslam_datastructure::Frame::SharedPtr>> weight_projections;
      for (const auto& [other_frame, count] : projection_counts) {
        // Skip the frames that have a projection count lower than the threshold
        if (count < min_projections) {
          continue;
        }

        const double weight = static_cast<double>(count)
                              / abs(static_cast<double>(frame->id()) - static_cast<double>(other_frame->id()));
        weight_projections.emplace_back(weight, other_frame);
      }

      // Sort and take the top k
      std::sort(
          weight_projections.begin(), weight_projections.end(),
          [](const std::pair<double, vslam_datastructure::Frame::SharedPtr>& lhs,
             const std::pair<double, vslam_datastructure::Frame::SharedPtr>& rhs) { return lhs.first > rhs.first; });
      if (weight_projections.size() > top_k_projections) {
        weight_projections.resize(top_k_projections);
      }

      vslam_datastructure::FrameSet projected_frames;
      for (auto& [_, frame] : weight_projections) {
        projected_frames.insert(frame);
      }

      return projected_frames;
    }

  }  // namespace utils
}  // namespace vslam_backend_plugins