// TODO: Licence

#ifndef VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_
#define VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
  namespace visualization_nodes {
    class RvizVisualNode : public rclcpp::Node {
    public:
      explicit RvizVisualNode(const rclcpp::NodeOptions &options);

    private:
      void live_frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);
      void update_frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr live_frame_sub_;
      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr update_frame_sub_;

      static constexpr int live_pose_marker_id_{0};

      int cam_marker_id_{1};

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr live_frame_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mappoint_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

      static const Eigen::Matrix3d cam_axes_transform_;
    };
  }  // namespace visualization_nodes
}  // namespace vslam_components

#endif  // VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_