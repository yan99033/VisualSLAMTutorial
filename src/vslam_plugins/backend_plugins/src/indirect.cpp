#include "vslam_backend_plugins/indirect.hpp"

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

namespace {
  Eigen::Vector3d cvPoint3dToEigenVector3d(const cv::Point3d& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); }

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

  cv::Mat eigenRotationTranslationToCvMat(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    return (cv::Mat_<double>(4, 4) << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), t.x(), R.coeff(1, 0), R.coeff(1, 1),
            R.coeff(1, 2), t.y(), R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), t.z(), 0.0, 0.0, 0.0, 1.0);
  }

  cv::Point3d eigenVector3dToCvPoint3d(const Eigen::Vector3d& vec) { return cv::Point3d(vec.x(), vec.y(), vec.z()); }

}  // namespace

namespace vslam_backend_plugins {
  Indirect::~Indirect() {
    exit_thread_ = true;

    if (local_ba_thread_.joinable()) {
      local_ba_condition_.notify_one();
      local_ba_thread_.join();
    }
  }

  void Indirect::initialize() { local_ba_thread_ = std::thread(&Indirect::local_ba_loop, this); }

  void Indirect::add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->is_keyframe());

    {
      std::lock_guard<std::mutex> kf_lck(keyframe_mutex_);
      keyframes_[keyframe->id()] = keyframe;
      current_keyframe_ = keyframe;
    }

    if (!run_local_ba_) {
      std::lock_guard<std::mutex> lck(local_ba_mutex_);
      run_local_ba_ = true;
      local_ba_condition_.notify_one();
    }
  }

  void Indirect::remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr Indirect::get_current_keyframe() {
    std::lock_guard<std::mutex> kf_lck(keyframe_mutex_);
    return current_keyframe_;
  }

  vslam_datastructure::Frame::SharedPtr Indirect::get_keyframe(const long unsigned int id) const {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(id) != keyframes_.end()) {
      return keyframes_.at(id);
    } else {
      return nullptr;
    }
  }

  void Indirect::local_ba_loop() {
    while (!exit_thread_) {
      {
        std::unique_lock<std::mutex> lck(local_ba_mutex_);
        local_ba_condition_.wait(lck, [this] { return exit_thread_ || run_local_ba_; });
      }

      if (exit_thread_) {
        break;
      }

      auto [core_kfs, core_mps] = get_core_keyframes_mappoints();
      if (core_kfs.size() < num_core_kfs_) {
        continue;
      }

      run_local_ba(core_kfs, core_mps);

      run_local_ba_ = false;
    }
  }

  std::pair<Indirect::CoreKfsSet, Indirect::CoreMpsSet> Indirect::get_core_keyframes_mappoints() {
    assert(num_core_kfs_ > 0);

    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    CoreKfsSet core_keyframes;
    CoreMpsSet core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    while (kfs_it != keyframes_.rend() && core_keyframes.size() < num_core_kfs_) {
      auto kf = kfs_it->second;

      if (!kf.get()) {
        continue;
      }

      core_keyframes.insert(kf.get());

      for (auto pt : kf->get_points()) {
        if (pt->has_mappoint() && !pt->get_mappoint()->is_outlier()
            && pt->get_mappoint()->get_projections().size() > 1) {
          core_mappoints.insert(pt->get_mappoint().get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void Indirect::run_local_ba(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver
        = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver
        = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
    optimizer.setAlgorithm(solver);

    const auto fixed_kf_id = current_keyframe_->id();

    // Create vertices and edges
    std::map<g2o::VertexPointXYZ*, vslam_datastructure::MapPoint*> core_mp_vertices;
    std::map<g2o::VertexSE3Expmap*, vslam_datastructure::Frame*> core_kf_vertices;
    std::list<g2o::EdgeSE3ProjectXYZ*> all_edges;
    std::set<vslam_datastructure::Frame*> non_core_kfs;
    std::map<vslam_datastructure::Frame*, g2o::VertexSE3Expmap*> existing_kf_vertices;
    unsigned long int vertex_edge_id{0};
    for (auto mp : core_mappoints) {
      if (mp == nullptr || mp->is_outlier()) {
        continue;
      }

      // Check if we have at least two valid projections
      int num_valid_projections{0};
      for (auto pt : mp->get_projections()) {
        if (pt == nullptr || !pt->has_frame()) {
          continue;
        }
        num_valid_projections++;
      }
      if (num_valid_projections < 2) {
        continue;
      }

      g2o::VertexPointXYZ* mp_vertex = new g2o::VertexPointXYZ();
      core_mp_vertices[mp_vertex] = mp;
      mp_vertex->setEstimate(cvPoint3dToEigenVector3d(mp->get_pos()));
      mp_vertex->setId(vertex_edge_id++);
      mp_vertex->setMarginalized(true);
      optimizer.addVertex(mp_vertex);

      // Projections
      for (auto pt : mp->get_projections()) {
        if (pt == nullptr || !pt->has_frame()) {
          continue;
        }

        // Keyframe vertex
        g2o::VertexSE3Expmap* kf_vertex;
        if (existing_kf_vertices.find(pt->get_frame()) != existing_kf_vertices.end()) {
          kf_vertex = existing_kf_vertices[pt->get_frame()];
        } else {
          kf_vertex = new g2o::VertexSE3Expmap();
          existing_kf_vertices[pt->get_frame()] = kf_vertex;
          kf_vertex->setEstimate(cvMatToSE3Quat(pt->get_frame()->T_f_w()));
          kf_vertex->setId(vertex_edge_id++);
          if (core_keyframes.find(pt->get_frame()) != core_keyframes.end()) {
            // Core keyframes
            core_kf_vertices[kf_vertex] = pt->get_frame();
            kf_vertex->setFixed(pt->get_frame()->id() == fixed_kf_id);
          } else {
            // Non-core keyframes
            non_core_kfs.insert(pt->get_frame());
            kf_vertex->setFixed(true);
          }
          optimizer.addVertex(kf_vertex);
        }

        // Projection
        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        e->setId(vertex_edge_id++);
        e->setVertex(0, mp_vertex);
        e->setVertex(1, kf_vertex);
        e->setMeasurement(Eigen::Vector2d(pt->keypoint.pt.x, pt->keypoint.pt.y));
        e->setInformation(Eigen::Matrix2d::Identity());  // TODO: incorporate uncertainty

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(huber_kernel_delta_);
        e->setRobustKernel(rk);

        const cv::Mat K = pt->get_frame()->K();

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
    optimizer.optimize(15);

    // If the errors after optimization is larger than before
    const double final_errs = optimizer.activeChi2();
    if (init_errs < final_errs) {
      return;
    }

    // Update keyframes
    for (auto [kf_vertex, kf_p] : core_kf_vertices) {
      auto T_f_w = kf_vertex->estimate();
      cv::Mat cv_T_f_w = eigenRotationTranslationToCvMat(T_f_w.rotation().toRotationMatrix(), T_f_w.translation());

      if (kf_p) {
        kf_p->set_pose(cv_T_f_w);
      }
    }

    for (auto e : all_edges) {
      if (!e->isDepthPositive() || e->chi2() > huber_kernel_delta_sq_) {
        auto mp_vertex = static_cast<g2o::VertexPointXYZ*>(e->vertex(0));
        auto mp_p = core_mp_vertices[mp_vertex];

        if (mp_p) {
          mp_p->set_outlier();
        }
      }
    }

    // Update map points
    int inlier_mappoints{0};
    for (auto [mp_vertex, mp_p] : core_mp_vertices) {
      auto mp = mp_vertex->estimate();
      if (mp.hasNaN()) {
        mp_p->set_outlier();
        continue;
      }

      if (mp_p && !mp_p->is_outlier()) {
        auto pt_3d = eigenVector3dToCvPoint3d(mp);

        mp_p->set_pos(pt_3d);
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
        kf1->add_T_this_other_kf(kf2, T_1_2);
      }
    }
  }

  void Indirect::add_loop_constraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
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
    }

    loop_optimization_running_ = true;

    run_pose_graph_optimization(kf_id_1, kf_id_2, T_1_2, sim3_scale);

    loop_optimization_running_ = false;
  }

  void Indirect::run_pose_graph_optimization(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                             const cv::Mat& T_1_2, const double sim3_scale) {
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(  // OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver);

    const auto fixed_kf_id = current_keyframe_->id();

    // Create vertices
    unsigned long int vertex_edge_id{0};
    std::map<long unsigned int, g2o::VertexSim3Expmap*> kf_vertices;
    for (const auto& [kf_id, kf] : keyframes_) {
      g2o::VertexSim3Expmap* v_sim3 = new g2o::VertexSim3Expmap();
      v_sim3->setId(vertex_edge_id++);
      v_sim3->setEstimate(cvMatToSim3(kf->T_f_w(), 1.0));
      v_sim3->setFixed(kf_id == fixed_kf_id);
      v_sim3->setMarginalized(false);
      optimizer.addVertex(v_sim3);

      kf_vertices[kf_id] = v_sim3;

      if (kf_id >= fixed_kf_id) {
        break;
      }
    }

    // Create edges
    for (const auto& [kf_id, kf] : keyframes_) {
      auto v_sim3_this = kf_vertices.at(kf_id);

      for (const auto& [other_kf, T_this_other] : kf->get_T_this_other_kfs()) {
        if (kf_vertices.find(other_kf->id()) == kf_vertices.end()) {
          continue;
        }

        auto v_sim3_other = kf_vertices.at(other_kf->id());

        g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
        e_sim3->setId(vertex_edge_id++);
        e_sim3->setVertex(0, v_sim3_this);
        e_sim3->setVertex(1, v_sim3_other);
        e_sim3->setMeasurement(cvMatToSim3(T_this_other, 1.0).inverse());
        e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

        optimizer.addEdge(e_sim3);
      }

      if (kf_id >= fixed_kf_id) {
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
    e_sim3->setMeasurement(cvMatToSim3(T_1_2, sim3_scale).inverse());
    e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();

    optimizer.addEdge(e_sim3);

    // Optimize pose graph
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    const double init_errs = optimizer.activeChi2();
    optimizer.optimize(15);

    // If the errors after optimization is larger than before
    const double final_errs = optimizer.activeChi2();
    if (init_errs < final_errs) {
      return;
    }

    // Recalculate SE(3) poses and map points in their host keyframe
    for (const auto [kf_id, kf_vertex] : kf_vertices) {
      // calculate the pose and scale
      const auto g2o_S_f_w = kf_vertex->estimate();
      const cv::Mat temp_T_f_w
          = eigenRotationTranslationToCvMat(g2o_S_f_w.rotation().toRotationMatrix(), g2o_S_f_w.translation());
      const double scale = g2o_S_f_w.scale();

      cv::Mat T_f_w = temp_T_f_w.clone();
      T_f_w.rowRange(0, 3).colRange(3, 4) /= scale;
      cv::Mat S_f_w = temp_T_f_w.clone();
      S_f_w.rowRange(0, 3).colRange(0, 3) *= scale;

      auto kf = keyframes_.at(kf_id);

      kf->update_sim3_pose_and_mps(S_f_w, T_f_w);
    }

    auto kfs_it = keyframes_.find(fixed_kf_id);
    while (kfs_it != keyframes_.begin()) {
      auto this_kf = kfs_it->second;
      // Update the relative constraints (T_this_others) in the keyframes using the edge constraints
      for (auto [other_kf, old_T_this_other] : this_kf->get_T_this_other_kfs()) {
        const cv::Mat T_this_other = this_kf->T_f_w() * other_kf->T_w_f();
        this_kf->add_T_this_other_kf(other_kf, T_this_other);
      }

      kfs_it--;
    }

    // Add the loop constraint
    auto kf_1 = keyframes_.at(kf_id_1);
    auto kf_2 = keyframes_.at(kf_id_2);
    const cv::Mat T_1_2_updated = kf_1->T_f_w() * kf_2->T_w_f();
    kf_1->add_T_this_other_kf(kf_2.get(), T_1_2_updated);
    kf_2->add_T_this_other_kf(kf_1.get(), T_1_2_updated.inv());
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)