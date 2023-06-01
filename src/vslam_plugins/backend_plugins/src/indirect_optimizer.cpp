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
  }

  void IndirectOptimizer::initialize() { local_ba_thread_ = std::thread(&IndirectOptimizer::local_ba_loop, this); }

  void IndirectOptimizer::add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->is_keyframe());

    {
      std::lock_guard<std::mutex> kf_lck(keyframe_mutex_);
      keyframes_[keyframe->id()] = keyframe;
      current_keyframe_ = keyframe;
    }

    if (!run_local_ba_) {
      run_local_ba_ = true;
      std::unique_lock<std::mutex> lck(local_ba_mutex_);
      local_ba_condition_.notify_one();
    }
  }

  void IndirectOptimizer::remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr IndirectOptimizer::get_current_keyframe() {
    std::lock_guard<std::mutex> kf_lck(keyframe_mutex_);
    return current_keyframe_;
  }

  vslam_datastructure::Frame::SharedPtr IndirectOptimizer::get_keyframe(const long unsigned int id) const {
    std::lock_guard<std::mutex> lck(keyframe_mutex_);
    if (keyframes_.find(id) != keyframes_.end()) {
      return keyframes_.at(id);
    } else {
      return nullptr;
    }
  }

  void IndirectOptimizer::local_ba_loop() {
    while (!exit_thread_) {
      {
        std::unique_lock<std::mutex> lck(local_ba_mutex_);
        local_ba_condition_.wait(lck, [this] { return exit_thread_ || run_local_ba_; });
      }

      if (exit_thread_) {
        break;
      }

      auto [core_keyframes, core_mappoints] = get_core_keyframes_mappoints();
      if (core_keyframes.size() < num_core_kfs_) {
        std::this_thread::yield();
        continue;
      }

      const long unsigned int current_kf_id = [&] {
        std::lock_guard<std::mutex> lck(keyframe_mutex_);
        return current_keyframe_->id();
      }();

      run_bundle_adjustment_impl(core_keyframes, core_mappoints, current_kf_id);

      run_local_ba_ = false;
    }
  }

  std::pair<IndirectOptimizer::CoreKfsSet, IndirectOptimizer::CoreMpsSet>
  IndirectOptimizer::get_core_keyframes_mappoints() {
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

      for (auto pt : kf->points()) {
        if (pt->has_mappoint() && pt->mappoint()->projections().size() > 1) {
          core_mappoints.insert(pt->mappoint().get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void IndirectOptimizer::add_loop_constraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
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

    const long unsigned int current_kf_id = [&] {
      std::lock_guard<std::mutex> lck(keyframe_mutex_);
      return current_keyframe_->id();
    }();

    run_pose_graph_optimization_impl(kf_id_1, kf_id_2, T_1_2, sim3_scale, keyframes_, current_kf_id);

    loop_optimization_running_ = false;
  }

  std::vector<vslam_msgs::msg::Frame> IndirectOptimizer::get_all_keyframe_msgs() const {
    std::vector<vslam_msgs::msg::Frame> keyframe_msgs;
    for (const auto& [_, kf] : keyframes_) {
      vslam_msgs::msg::Frame keyframe_msg;
      kf->to_msg(&keyframe_msg, true);
      keyframe_msgs.push_back(keyframe_msg);
    }

    return keyframe_msgs;
  }

  // void IndirectOptimizer::remove_outlier_mappoints() {
  //   for (auto& [_, kf] : keyframes_) {
  //     if
  //   }
  // }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::IndirectOptimizer, vslam_backend::base::Backend)