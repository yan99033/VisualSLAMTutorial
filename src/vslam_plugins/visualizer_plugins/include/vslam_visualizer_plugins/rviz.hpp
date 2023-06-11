#ifndef VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_
#define VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_visualizer_plugins/visualizer.hpp"

namespace vslam_visualizer_plugins {

  class RViz : public Visualizer {
  public:
    ~RViz();

    void initialize() override;

    void add_live_frame(const vslam_msgs::msg::Frame& frame_msg) override;

    void add_keyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    void remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    void replace_all_keyframes(const FrameVec& frame_msgs) override;

    inline std::string get_plugin_name() override { return "vslam_visualizer_plugins::RViz"; }

  private:
    /// Visualizer node
    rclcpp::Node::SharedPtr node_{nullptr};

    /// Image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    /// Live frame publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr live_frame_publisher_;

    /// Keyframe publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keyframe_publisher_;

    /// Map point publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mappoint_publisher_;

    /// Camera pose publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// The frame id of the global map frame
    std::string map_frame_id_{"map"};

    /// The frame id of the camera frame
    std::string cam_frame_id_{"cam"};

    /// Live camera marker scale
    double live_cam_marker_scale_{10.0};

    /// Live camera marker namespace
    std::string live_cam_marker_ns_{"live"};

    /// Live camera marker colour
    std::array<double, 3> live_cam_marker_rgb_{1.0, 0.0, 0.0};

    /// Scale of keyframe and map point markers
    double marker_scale_{0.25};

    /// Keyframe camera marker namespace
    std::string keyframe_cam_marker_ns_{"keyframe_poses"};

    std::array<double, 3> keyframe_cam_marker_rgb_{0.0, 1.0, 0.0};

    std::string keyframe_mps_marker_ns_{"keyframe_mps"};

    std::array<double, 3> mappoint_marker_rgb_{0.0, 0.0, 0.0};

    double line_thickness_{1.0};
  };
}  // namespace vslam_visualizer_plugins

#endif  // VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_
