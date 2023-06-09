#ifndef VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_
#define VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_camera_tracker {
  namespace base {
    class CameraTracker : public virtual vslam_plugin::base::Plugin {
    public:
      virtual void initialize() = 0;

      /// Track the relative transformation (T_2_1) given a vector containing the 2D-2D correspondences between frame1
      /// and frame2
      /**
       * The tracking quality is indicated by the number of inlier points relative to the total amount of
       * correspondences provided
       *
       * \param[in] matched_points a vector containing the 2D-2D correspondences between frame1 and frame2
       * \param[out] T_2_1 the camera transformation from frame2 to frame1
       * \return a boolean indicating the tracking quality
       */
      virtual bool track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                     cv::Mat& T_2_1)
          = 0;

      /// Track the relative transformation (T_2_1) given a vector containing the 3D-2D correspondences between frame1
      /// and frame2
      /**
       * The tracking quality is indicated by the number of inlier points relative to the total amount of
       * correspondences provided
       *
       * \param[in] matched_points a vector containing the 3D-2D correspondences between frame1 and frame2
       * \param[out] T_2_1 the camera transformation from frame2 to frame1
       * \return a boolean indicating the tracking quality
       */
      virtual bool track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                     cv::Mat& T_2_1)
          = 0;
      virtual ~CameraTracker() {}

    protected:
      CameraTracker() {}
    };
  }  // namespace base
}  // namespace vslam_camera_tracker

#endif  // VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_