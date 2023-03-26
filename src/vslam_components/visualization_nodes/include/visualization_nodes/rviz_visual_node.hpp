// TODO: Licence

#ifndef VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_
#define VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
namespace visualization_nodes {
class RvizVisualNode : public rclcpp::Node {
public:
  explicit RvizVisualNode(const rclcpp::NodeOptions &options);

private:
  void frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);

  rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
};
} // namespace visualization_nodes
} // namespace vslam_components

#endif // VISUALIZATION_NODES__RVIZ_VISUAL_NODE_HPP_