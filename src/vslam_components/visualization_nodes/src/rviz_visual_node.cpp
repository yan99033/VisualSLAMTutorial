#include "visualization_nodes/rviz_visual_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

namespace {
int encoding2mat_type(const std::string &encoding) {
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}

void calculateLivePoseMarkers(
    const geometry_msgs::msg::Pose &pose,
    visualization_msgs::msg::Marker &pose_marker,
    const Eigen::Matrix3d cam_axes_transform = Eigen::Matrix3d::Identity()) {
  // Convert the pose to an Eigen matrix
  Eigen::Isometry3d eigen_transform;
  tf2::fromMsg(pose, eigen_transform);

  eigen_transform.translation() =
      cam_axes_transform * eigen_transform.translation();
  eigen_transform.linear() = cam_axes_transform * eigen_transform.rotation() *
                             cam_axes_transform.inverse();

  // Calculate the preset vertices
  constexpr double s = 5.0;
  constexpr double fx = 340.0;
  constexpr double fy = 350.0;
  constexpr double cx = 300.0;
  constexpr double cy = 200.0;
  constexpr double w = 600.0;
  constexpr double h = 400.0;
  constexpr double p0[] = {0.0, 0.0, 0.0, 1.0};
  constexpr double p1[] = {s * (0 - cx) / fx, s * (0 - cy) / fy, s, 1.0};
  constexpr double p2[] = {s * (0 - cx) / fx, s * (h - 1 - cy) / fy, s, 1.0};
  constexpr double p3[] = {s * (w - 1 - cx) / fx, s * (h - 1 - cy) / fy, s,
                           1.0};
  constexpr double p4[] = {s * (w - 1 - cx) / fx, s * (0 - cy) / fy, s, 1.0};

  constexpr int num_vertices = 16;
  constexpr int num_dims = 4;
  Eigen::MatrixXd marker_vertices(num_dims, num_vertices);
  marker_vertices << p0[0], p1[0], p0[0], p2[0], p0[0], p3[0], p0[0], p4[0],
      p4[0], p3[0], p3[0], p2[0], p2[0], p1[0], p1[0], p4[0], //
      p0[1], p1[1], p0[1], p2[1], p0[1], p3[1], p0[1], p4[1], p4[1], p3[1],
      p3[1], p2[1], p2[1], p1[1], p1[1], p4[1], //
      p0[2], p1[2], p0[2], p2[2], p0[2], p3[2], p0[2], p4[2], p4[2], p3[2],
      p3[2], p2[2], p2[2], p1[2], p1[2], p4[2], //
      p0[3], p1[3], p0[3], p2[3], p0[3], p3[3], p0[3], p4[3], p4[3], p3[3],
      p3[3], p2[3], p2[3], p1[3], p1[3], p4[3]; //

  marker_vertices.topRows(3) = cam_axes_transform * marker_vertices.topRows(3);

  const auto vertices_transformed = eigen_transform.matrix() * marker_vertices;

  auto marker = visualization_msgs::msg::Marker();

  // Add vertices to the marker msg
  pose_marker.header.frame_id = "map";
  pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  pose_marker.action = visualization_msgs::msg::Marker::ADD;
  pose_marker.scale.x = 0.1;
  pose_marker.color.g = 1.0;
  pose_marker.color.a = 1.0;

  for (int i = 0; i < num_vertices; i++) {
    auto pt = geometry_msgs::msg::Point();
    pt.x = vertices_transformed.col(i)(0);
    pt.y = vertices_transformed.col(i)(1);
    pt.z = vertices_transformed.col(i)(2);

    pose_marker.points.push_back(pt);
  }
}
} // namespace

namespace vslam_components {

namespace visualization_nodes {

const Eigen::Matrix3d RvizVisualNode::cam_axes_transform_ =
    (Eigen::Matrix3d() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)
        .finished();

RvizVisualNode::RvizVisualNode(const rclcpp::NodeOptions &options)
    : Node("rviz_visual_node", options) {
  // Frame subscriber and publisher
  frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
      "in_frame", 10, std::bind(&RvizVisualNode::frame_callback, this, _1));
  live_frame_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("live_frame_marker", 1);
}

void RvizVisualNode::frame_callback(
    vslam_msgs::msg::Frame::UniquePtr frame_msg) {
  RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

  visualization_msgs::msg::Marker pose_marker;
  calculateLivePoseMarkers(frame_msg->pose, pose_marker, cam_axes_transform_);
  pose_marker.id = live_pose_marker_id_;
  live_frame_publisher_->publish(pose_marker);
}
} // namespace visualization_nodes
} // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    vslam_components::visualization_nodes::RvizVisualNode)