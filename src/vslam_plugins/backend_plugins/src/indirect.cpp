#include "vslam_backend_plugins/indirect.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

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

  std::pair<Indirect::CoreKfs, Indirect::CoreMps> Indirect::get_core_keyframes_mappoints() {
    assert(num_core_kfs_ > 0);

    CoreKfs core_keyframes;
    CoreMps core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    for (size_t i = 0; i < num_core_kfs_; i++) {
      auto kf = kfs_it->second;
      core_keyframes.insert(kf);

      // TODO: make it thread-safe
      for (auto pt : *(kf->get_points())) {
        if (pt->mappoint.get() && !pt->mappoint->is_outlier) {
          core_mappoints.insert(pt->mappoint);
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void Indirect::run_local_ba(CoreKfs& core_keyframes, CoreMps& core_mappoints) {
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

    // Create keyframe vertices

    // Create map point vertices

    // Create projection edges

    // optimize graph

    // Update keyframes and map points
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)