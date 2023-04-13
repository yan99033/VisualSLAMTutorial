#include "vslam_backend_plugins/indirect.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

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
}  // namespace

namespace vslam_backend_plugins {
  Indirect::~Indirect() {
    exit_thread_ = true;

    if (local_ba_thread_.joinable()) {
      local_ba_condition_.notify_one();
      local_ba_thread_.join();
    }
  }

  void Indirect::initialize(const cv::Mat& K) {
    K_ = K;
    local_ba_thread_ = std::thread(&Indirect::local_ba_loop, this);
  }

  void Indirect::add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->is_keyframe());

    keyframes_[keyframe->id()] = keyframe;

    current_keyframe_ = keyframe;

    {
      std::lock_guard<std::mutex> lck(local_ba_mutex_);
      run_local_ba_ = true;
      local_ba_condition_.notify_one();
    }
  }

  void Indirect::remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr Indirect::get_current_keyframe() { return current_keyframe_; }

  void Indirect::local_ba_loop() {
    while (!exit_thread_) {
      {
        std::unique_lock<std::mutex> lck(local_ba_mutex_);
        local_ba_condition_.wait(lck, [this] { return exit_thread_ || run_local_ba_; });
      }

      if (exit_thread_) {
        break;
      }

      if (keyframes_.size() < num_core_kfs_) {
        continue;
      }

      auto [core_kfs, core_mps] = get_core_keyframes_mappoints();

      run_local_ba(core_kfs, core_mps);

      run_local_ba_ = false;
    }
    std::cout << "terminated local BA" << std::endl;
  }

  std::pair<Indirect::CoreKfsMap, Indirect::CoreMpsSet> Indirect::get_core_keyframes_mappoints() {
    assert(num_core_kfs_ > 0);

    CoreKfsMap core_keyframes;
    CoreMpsSet core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    // for (size_t i = 0; i < num_core_kfs_; i++) {
    while (kfs_it != keyframes_.rend() && core_keyframes.size() < num_core_kfs_) {
      auto kf = kfs_it->second;

      if (kf.get() == nullptr) {
        continue;
      }

      core_keyframes[kfs_it->first] = kf.get();

      // TODO: make it thread-safe
      for (auto pt : *(kf->get_points())) {
        if (pt->mappoint.get() && !pt->mappoint->is_outlier) {
          core_mappoints.insert(pt->mappoint.get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void Indirect::run_local_ba(CoreKfsMap& core_keyframes, CoreMpsSet& core_mappoints) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::string solver_name;
#ifdef G2O_HAVE_CHOLMOD
    solver_name = "lm_fix6_3_cholmod";
#else
    solver_name = "lm_fix6_3";
#endif;
    g2o::OptimizationAlgorithmProperty solver_property;
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solver_name, solver_property));

    constexpr float huber_kernel_delta{2.45};

    // Create vertices and edges
    std::map<vslam_datastructure::MapPoint*, g2o::VertexPointXYZ*> all_mp_vertices;
    std::map<vslam_datastructure::Frame*, g2o::VertexSE3Expmap*> all_kf_vertices;
    unsigned long int vertex_id{0};
    for (auto mp : core_mappoints) {
      if (mp == nullptr || mp->is_outlier || mp->projections.empty()) {
        continue;
      }

      g2o::VertexPointXYZ* mp_vertex = new g2o::VertexPointXYZ();
      all_mp_vertices[mp] = mp_vertex;
      mp_vertex->setEstimate(cvPoint3dToEigenVector3d(mp->pt_3d));
      mp_vertex->setId(vertex_id++);
      mp_vertex->setMarginalized(true);
      optimizer.addVertex(mp_vertex);

      // Projections
      for (auto pt : mp->projections) {
        if (pt == nullptr || pt->frame == nullptr) {
          continue;
        }

        // Keyframe vertex
        g2o::VertexSE3Expmap* kf_vertex = new g2o::VertexSE3Expmap();
        all_kf_vertices[pt->frame] = kf_vertex;
        kf_vertex->setEstimate(cvMatToSE3Quat(pt->frame->get_pose()));
        kf_vertex->setId(vertex_id++);
        if (core_keyframes.find(pt->frame->id()) != core_keyframes.end()) {
          // Core keyframes
          kf_vertex->setFixed(false);
        } else {
          // Non-core keyframes
          kf_vertex->setFixed(true);
        }
        optimizer.addVertex(kf_vertex);

        // Projection
        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        e->setVertex(0, mp_vertex);
        e->setVertex(1, kf_vertex);
        e->setMeasurement(Eigen::Vector2d(pt->keypoint.pt.x, pt->keypoint.pt.y));
        e->setInformation(Eigen::Matrix2d::Identity());

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(huber_kernel_delta);

        e->fx = K_.at<double>(0, 0);
        e->fy = K_.at<double>(1, 1);
        e->cx = K_.at<double>(0, 2);
        e->cy = K_.at<double>(1, 2);

        optimizer.addEdge(e);
      }
    }

    // optimize graph
    optimizer.initializeOptimization();
    optimizer.optimize(40);

    // Update keyframes and map points
    for (const auto [kf_pt, kf_vertex] : all_kf_vertices) {
      std::cout << "old" << std::endl;
      std::cout << kf_pt->get_pose();

      std::cout << "new" << std::endl;
      auto pose = kf_vertex->estimate();
      std::cout << pose.rotation().toRotationMatrix() << std::endl;
      std::cout << pose.translation() << std::endl;
    }
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)