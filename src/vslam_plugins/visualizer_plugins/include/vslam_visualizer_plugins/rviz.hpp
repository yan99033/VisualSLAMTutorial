#ifndef VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_
#define VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/visualizer.hpp"

namespace vslam_visualizer_plugins {

  class RViz : public vslam_visualizer_base::Visualizer {
  public:
    ~RViz();

    void initialize() override;

    void add_live_frame(const vslam_msgs::msg::Frame& frame_msg) override;

    void add_keyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    void remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    void replace_all_keyframes(const FrameVec& frame_msgs) override;

  private:
    rclcpp::Node::SharedPtr node_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr live_frame_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keyframe_publisher_;

    std::string frame_id_{"map"};

    double marker_scale_{0.25};

    double line_thickness_{1.0};

    // Convert the camera coordinate frame (x: right, y: down, z: forward) to the `map` coordinate frame (x: forward,
    // y: left, z: up)
    // @sa https://www.ros.org/reps/rep-0105.html
    static const Eigen::Matrix3d cam_axes_transform_;
  };
}  // namespace vslam_visualizer_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
