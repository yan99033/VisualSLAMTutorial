#ifndef VSLAM_VISUALIZER_PLUGINS__VISUALIZER_HPP_
#define VSLAM_VISUALIZER_PLUGINS__VISUALIZER_HPP_

#include <Eigen/Dense>

#include "vslam_plugins_base/visualizer.hpp"

using Mat3x16 = Eigen::Matrix<double, 3, 16>;
using Mat4x16 = Eigen::Matrix<double, 4, 16>;

namespace vslam_visualizer_plugins {
  namespace abstract {
    class Visualizer : public virtual vslam_visualizer::base::Visualizer {
    protected:
      virtual Mat3x16 get_camera_vertices(const double scale,
                                          const Eigen::Isometry3d& T_w_c = Eigen::Isometry3d::Identity())
          = 0;
    };
  }  // namespace abstract

  class Visualizer : public virtual abstract::Visualizer {
  protected:
    Mat3x16 get_camera_vertices(const double scale,
                                const Eigen::Isometry3d& T_w_c = Eigen::Isometry3d::Identity()) override;

    // Convert the camera coordinate frame (x: right, y: down, z: forward) to the `map` coordinate frame (x: forward,
    // y: left, z: up)
    // @sa https://www.ros.org/reps/rep-0105.html
    static const Eigen::Matrix3d cam_axes_transform_;

    // Camera marker vertices at identity
    static constexpr const double fx = 340.0;
    static constexpr const double fy = 350.0;
    static constexpr const double cx = 300.0;
    static constexpr const double cy = 200.0;
    static constexpr const double w = 600.0;
    static constexpr const double h = 400.0;
    static constexpr const double p0[] = {0.0, 0.0, 0.0, 1.0};
    static constexpr const double p1[] = {(0 - cx) / fx, (0 - cy) / fy, 1.0, 1.0};
    static constexpr const double p2[] = {(0 - cx) / fx, (h - 1 - cy) / fy, 1.0, 1.0};
    static constexpr const double p3[] = {(w - 1 - cx) / fx, (h - 1 - cy) / fy, 1.0, 1.0};
    static constexpr const double p4[] = {(w - 1 - cx) / fx, (0 - cy) / fy, 1.0, 1.0};
    static constexpr const int num_vertices_{16};
    static constexpr const int num_dims_{4};
    static const Mat4x16 cam_marker_vertices_;
  };
}  // namespace vslam_visualizer_plugins

#endif  //  VSLAM_VISUALIZER_PLUGINS__VISUALIZER_HPP_