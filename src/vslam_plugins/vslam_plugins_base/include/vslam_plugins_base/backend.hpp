#ifndef VSLAM_PLUGINS_BASE__BACKEND_HPP_
#define VSLAM_PLUGINS_BASE__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_backend {
  namespace base {
    class Backend : public virtual vslam_plugin::base::Plugin {
    public:
      /// Back-end initializer
      virtual void initialize() = 0;

      /// Add a new keyframe
      /**
       * \param[in] frame a new keyframe
       */
      virtual void addKeyfame(vslam_datastructure::Frame::SharedPtr frame) = 0;

      /// Remove a keyframe
      /**
       * \param[in] frame the keyframe to be removed
       */
      virtual void removeKeyframe(vslam_datastructure::Frame::SharedPtr frame) = 0;

      /// Get a keyframe using the id. Return a nullptr if the keyframe cannot be found
      virtual vslam_datastructure::Frame::SharedPtr getKeyframe(const long unsigned int id) const = 0;

      /// Add a loop constraint and run pose-graph optimization
      /**
       * \param[in] kf_id_1 the first keyframe id
       * \param[in] kf_id_2 the second keyframe id
       * \param[in] T_1_2 relative transformation from the first to the second keyframe
       * \param[in] sim3_scale the similarity transform scale
       */
      virtual void addLoopConstraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                     const cv::Mat& T_1_2, const double sim3_scale)
          = 0;

      /// Convert all the keyframes to frame msgs to refresh the visualizer
      virtual std::vector<vslam_msgs::msg::Frame> getAllKeyframeMsgs() const = 0;

      /// Destructor
      virtual ~Backend() {}

    protected:
      /// Constructor
      Backend() {}
    };
  }  // namespace base
}  // namespace vslam_backend

#endif  // VSLAM_PLUGINS_BASE__BACKEND_HPP_