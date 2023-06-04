#ifndef VSLAM_BACKEND_PLUGINS__BACKEND_HPP_
#define VSLAM_BACKEND_PLUGINS__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {
  template <typename T> struct Cmp {
    bool operator()(const T* lhs, const T* rhs) const { return lhs->id() < rhs->id(); }
  };

  namespace abstract {
    class Optimizer : public virtual vslam_backend::base::Backend {
    protected:
      using CoreKfsSet = std::set<vslam_datastructure::Frame*, Cmp<vslam_datastructure::Frame>>;
      using CoreMpsSet = std::set<vslam_datastructure::MapPoint*, Cmp<vslam_datastructure::MapPoint>>;

      virtual void run_bundle_adjustment_impl(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints,
                                              const long unsigned int current_kf_id)
          = 0;

      virtual void run_pose_graph_optimization_impl(
          const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
          const double sim3_scale, std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr>& keyframes,
          const long unsigned int current_kf_id)
          = 0;
    };
  }  // namespace abstract

  class Optimizer : public virtual abstract::Optimizer {
  protected:
    ~Optimizer();

    void run_bundle_adjustment_impl(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints,
                                    const long unsigned int current_kf_id) override;

    void run_pose_graph_optimization_impl(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                          const cv::Mat& T_1_2, const double sim3_scale,
                                          std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr>& keyframes,
                                          const long unsigned int current_kf_id) override;

    // All keyframes (can be looked up using their id)
    std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr> keyframes_;

    // The keyframe that is closest to the current camera pose
    vslam_datastructure::Frame::SharedPtr current_keyframe_;

    /// The window size of the error
    static constexpr const double huber_kernel_delta_{2.4477};

    /// A squared error above this is considered as outlier in the data
    static constexpr const double huber_kernel_delta_sq_{5.991};

    /// Bundle adjustment optimization iterations
    static constexpr const int ba_iterations_{15};

    /// Pose-graph optimization iterations
    static constexpr const int pgo_iterations_{15};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__BACKEND_HPP_