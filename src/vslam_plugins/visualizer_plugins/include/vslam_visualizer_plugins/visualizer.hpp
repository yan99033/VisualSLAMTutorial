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
      /// Get the camera vertices given the camera scale and camera pose to visualize the camera's field of view
      /**
       * \param[in] scale the scale of the camera visually
       * \param[in] T_w_c the camera pose
       * \return 16 vertices, where lines can be formed between 0-1, 1-2, 2-3, etc. to draw a camera marker
       */
      virtual Mat3x16 getCameraVertices(const double scale,
                                        const Eigen::Isometry3d& T_w_c = Eigen::Isometry3d::Identity())
          = 0;
    };
  }  // namespace abstract

  class Visualizer : public virtual abstract::Visualizer {
  protected:
    /// Get the camera vertices given the camera scale and camera pose to visualize the camera's field of view
    /**
     * \param[in] scale the scale of the camera visually
     * \param[in] T_w_c the camera pose
     * \return 16 vertices, where lines can be formed between 0-1, 1-2, 2-3, etc. to draw a camera marker
     */
    Mat3x16 getCameraVertices(const double scale,
                              const Eigen::Isometry3d& T_w_c = Eigen::Isometry3d::Identity()) override;

    // Transformation matrix to convert the camera coordinate frame (x: right, y: down, z: forward) to the `map`
    // coordinate frame (x: forward, y: left, z: up)
    /**
     * \sa https://www.ros.org/reps/rep-0105.html
     */
    static const Eigen::Matrix3d cam_axes_transform_;

    /// focal length x to draw the camera marker at the identity pose
    static constexpr const double fx = 340.0;

    /// focal length y to draw the camera marker at the identity pose
    static constexpr const double fy = 350.0;

    /// Camera principal point x to draw the camera marker at the identity pose
    static constexpr const double cx = 300.0;

    /// Camera principal point y to draw the camera marker at the identity pose
    static constexpr const double cy = 200.0;

    /// Image width to draw the camera marker at the identity pose
    static constexpr const double w = 600.0;

    /// Image height to draw the camera marker at the identity pose
    static constexpr const double h = 400.0;

    /// Pre-computed camera vertex 0
    static constexpr const double p0[] = {0.0, 0.0, 0.0, 1.0};

    /// Pre-computed camera vertex 1
    static constexpr const double p1[] = {(0 - cx) / fx, (0 - cy) / fy, 1.0, 1.0};

    /// Pre-computed camera vertex 2
    static constexpr const double p2[] = {(0 - cx) / fx, (h - 1 - cy) / fy, 1.0, 1.0};

    /// Pre-computed camera vertex 3
    static constexpr const double p3[] = {(w - 1 - cx) / fx, (h - 1 - cy) / fy, 1.0, 1.0};

    /// Pre-computed camera vertex 4
    static constexpr const double p4[] = {(w - 1 - cx) / fx, (0 - cy) / fy, 1.0, 1.0};

    /// Number of camera vertices
    static constexpr const int num_vertices_{16};

    /// Number of dimensions for each camera vertex
    static constexpr const int num_dims_{4};

    /// Camera marker vertices
    static const Mat4x16 cam_marker_vertices_;
  };
}  // namespace vslam_visualizer_plugins

#endif  //  VSLAM_VISUALIZER_PLUGINS__VISUALIZER_HPP_