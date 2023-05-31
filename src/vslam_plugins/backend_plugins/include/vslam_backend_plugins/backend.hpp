#ifndef VSLAM_BACKEND_PLUGINS__BACKEND_HPP_
#define VSLAM_BACKEND_PLUGINS__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {
  namespace abstract {
    class Optimizer : virtual public vslam_backend::base::Backend {
    protected:
      using CoreKfsSet = std::set<vslam_datastructure::Frame*>;
      using CoreMpsSet = std::set<vslam_datastructure::MapPoint*>;

      virtual void run_bundle_adjustment_impl(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints,
                                              const long unsigned int fixed_kf_id)
          = 0;

      virtual void run_pose_graph_optimization_impl(
          const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
          const double sim3_scale, std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr>& keyframes,
          const long unsigned int fixed_kf_id)
          = 0;
    };
  }  // namespace abstract

  class Optimizer : virtual public abstract::Optimizer {
  protected:
    void run_bundle_adjustment_impl(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints,
                                    const long unsigned int fixed_kf_id) override;

    void run_pose_graph_optimization_impl(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                          const cv::Mat& T_1_2, const double sim3_scale,
                                          std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr>& keyframes,
                                          const long unsigned int fixed_kf_id) override;

    static constexpr const double huber_kernel_delta_{2.4477};
    static constexpr const double huber_kernel_delta_sq_{5.991};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__BACKEND_HPP_