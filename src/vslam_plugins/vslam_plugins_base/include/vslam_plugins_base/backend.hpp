#ifndef VSLAM_PLUGINS_BASE__BACKEND_HPP_
#define VSLAM_PLUGINS_BASE__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_backend {
  namespace base {
    class Backend {
    public:
      /// Initialize the back-end
      virtual void initialize() = 0;

      /// Add a new keyframe
      /**
       * \param[in] frame a new keyframe
       */
      virtual void add_keyframe(vslam_datastructure::Frame::SharedPtr frame) = 0;

      /// Remove a keyframe
      /**
       * \param[in] frame the keyframe to be removed
       */
      virtual void remove_keyframe(vslam_datastructure::Frame::SharedPtr frame) = 0;

      /// Get the keyframe by its id
      virtual vslam_datastructure::Frame::SharedPtr get_keyframe(const long unsigned int id) const = 0;

      /// Add a loop constraint and run pose-graph optimization
      /**
       * \param[in] kf_id_1 the first keyframe id
       * \param[in] kf_id_2 the second keyframe id
       * \param[in] T_1_2 relative transformation from the first to the second keyframe
       * \param[in] sim3_scale the similarity transform scale
       */
      virtual void add_loop_constraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                       const cv::Mat& T_1_2, const double sim3_scale)
          = 0;

      // Convert all the keyframes to frame msgs to refresh the visualizer
      virtual std::vector<vslam_msgs::msg::Frame> get_all_keyframe_msgs() const = 0;

      virtual ~Backend() {}

    protected:
      Backend() {}
    };
  }  // namespace base
}  // namespace vslam_backend

#endif  // VSLAM_PLUGINS_BASE__BACKEND_HPP_