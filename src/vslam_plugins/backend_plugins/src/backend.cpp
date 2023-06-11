#include "vslam_backend_plugins/backend.hpp"

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

#include "vslam_backend_plugins/utils.hpp"
#include "vslam_utils/converter.hpp"

namespace vslam_backend_plugins {
  Optimizer::~Optimizer() {
    keyframes_.clear();
    current_keyframe_.reset();
    std::cerr << "Terminated Optimizer" << std::endl;
  }

  void Optimizer::runBundleAdjustmentImpl(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints,
                                          const long unsigned int current_kf_id) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver
        = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
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
    std::set<vslam_datastructure::Frame*> non_core_kfs;
    std::map<vslam_datastructure::Frame*, g2o::VertexSE3Expmap*> existing_kf_vertices;
    unsigned long int vertex_edge_id{0};
    for (auto mp : core_mappoints) {
      if (mp == nullptr || mp->isOutlier()) {
        continue;
      }

      // Check if we have at least two valid projections
      int num_valid_projections{0};
      for (auto pt : mp->projections()) {
        if (pt == nullptr || !pt->hasFrame()) {
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
        if (pt == nullptr || !pt->hasFrame()) {
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
            kf_vertex->setFixed(pt->frame()->id() == current_kf_id || pt->frame()->active_tracking_state);
          } else {
            // Non-core keyframes
            non_core_kfs.insert(pt->frame());
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
        auto mp_vertex = static_cast<g2o::VertexPointXYZ*>(e->vertex(0));
        auto mp_p = core_mp_vertices[mp_vertex];

        if (mp_p) {
          mp_p->setOutlier();
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

    // Add relative pose constraints among the core keyframes
    for (auto kf1 : core_keyframes) {
      for (auto kf2 : core_keyframes) {
        if (kf1 == kf2) {
          continue;
        }

        // Relative pose constraint
        const cv::Mat T_1_2 = kf1->T_f_w() * kf2->T_w_f();
        kf1->addTThisOtherKf(kf2, T_1_2);
      }
    }

    for (auto core_kf : core_keyframes) {
      core_kf->active_ba_state = false;
    }
  }

  void Optimizer::runPoseGraphOptimizationImpl(
      const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2, const double sim3_scale,
      std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr>& keyframes,
      const long unsigned int current_kf_id) {
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(  // OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver);

    // Create vertices
    unsigned long int vertex_edge_id{0};
    std::map<long unsigned int, g2o::VertexSim3Expmap*> kf_vertices;
    for (const auto& [kf_id, kf] : keyframes) {
      g2o::VertexSim3Expmap* v_sim3 = new g2o::VertexSim3Expmap();
      v_sim3->setId(vertex_edge_id++);
      v_sim3->setEstimate(utils::cvMatToSim3(kf->T_f_w(), 1.0));
      v_sim3->setFixed(kf_id == current_kf_id);
      v_sim3->setMarginalized(false);
      optimizer.addVertex(v_sim3);

      kf_vertices[kf_id] = v_sim3;

      if (kf_id >= current_kf_id) {
        break;
      }
    }

    // Create edges
    for (const auto& [kf_id, kf] : keyframes) {
      auto v_sim3_this = kf_vertices.at(kf_id);

      for (const auto& [other_kf, T_this_other] : kf->TThisOtherKfs()) {
        if (kf_vertices.find(other_kf->id()) == kf_vertices.end()) {
          continue;
        }

        auto v_sim3_other = kf_vertices.at(other_kf->id());

        g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
        e_sim3->setId(vertex_edge_id++);
        e_sim3->setVertex(0, v_sim3_this);
        e_sim3->setVertex(1, v_sim3_other);
        e_sim3->setMeasurement(utils::cvMatToSim3(T_this_other, 1.0).inverse());
        e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

        optimizer.addEdge(e_sim3);
      }

      if (kf_id >= current_kf_id) {
        break;
      }
    }

    // The loop edge
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
      auto kf = keyframes.at(kf_id);

      // If the keyframe is being used or optimized
      if (kf->active_tracking_state || kf->active_ba_state) {
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

    auto kfs_it = keyframes.find(current_kf_id);
    while (kfs_it != keyframes.begin()) {
      auto this_kf = kfs_it->second;
      // Update the relative constraints (T_this_others) in the keyframes using the edge constraints
      for (auto [other_kf, old_T_this_other] : this_kf->TThisOtherKfs()) {
        const cv::Mat T_this_other = this_kf->T_f_w() * other_kf->T_w_f();
        this_kf->addTThisOtherKf(other_kf, T_this_other);
      }

      kfs_it--;
    }

    // Add the loop constraint
    auto kf_1 = keyframes.at(kf_id_1);
    auto kf_2 = keyframes.at(kf_id_2);
    const cv::Mat T_1_2_updated = kf_1->T_f_w() * kf_2->T_w_f();
    kf_1->addTThisOtherKf(kf_2.get(), T_1_2_updated);
    kf_2->addTThisOtherKf(kf_1.get(), T_1_2_updated.inv());
  }

}  // namespace vslam_backend_plugins