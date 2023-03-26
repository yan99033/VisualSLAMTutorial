#include "visualization_nodes/rviz_visual_node.hpp"

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

// convert coords
// # Convert from left-hand rule to right-hand rule coordinates system
// (visualization) # Right-hand rule: x forward, y left, z up # Left-hand rule:
// z forward, x right, y down convert_coords = True cam_axes_transform =
// np.array([[0.0 , 0.0 , 1.0],
//                                [-1.0, 0.0 , 0.0],
//                                [0.0 , -1.0, 0.0]]).astype(np.float)
//
// rot_mat_c = cam_axes_transform @ rot_mat @ np.linalg.inv(cam_axes_transform)
// trans_c = cam_axes_transform @ trans

// Create line markers that represent the camera pose (example code)
// auto marker = visualization_msgs::msg::Marker();
// marker.header.frame_id = "base_link";
// marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
// marker.action = visualization_msgs::msg::Marker::ADD;
// marker.pose.orientation.w = 1.0;
// marker.scale.x = 0.1;
// marker.color.r = 1.0;
// marker.color.a = 1.0;

// auto p1 = geometry_msgs::msg::Point();
// p1.x = 0;
// p1.y = 0;
// p1.z = 0;

// auto p2 = geometry_msgs::msg::Point();
// p2.x = 1;
// p2.y = 1;
// p2.z = 1;

// marker.points.push_back(p1);
// marker.points.push_back(p2);

// point pairs needed for drawing the camera frustum
// glVertex3f(0,0,0);
// glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
// glVertex3f(0,0,0);
// glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
// glVertex3f(0,0,0);
// glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
// glVertex3f(0,0,0);
// glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

// glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
// glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

// glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
// glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

// glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
// glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

// glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
// glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

} // namespace

namespace vslam_components {

namespace visualization_nodes {

RvizVisualNode::RvizVisualNode(const rclcpp::NodeOptions &options)
    : Node("rviz_visual_node", options) {
  // Frame subscriber and publisher
  frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
      "in_frame", 10, std::bind(&RvizVisualNode::frame_callback, this, _1));
  live_frame_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("live_frame_marker", 1);

  // TODO:
  // set frame id
  // create a constexpr for the left-hand to right-hand rule coordinate
  //   conversion matrix
}

void RvizVisualNode::frame_callback(
    vslam_msgs::msg::Frame::UniquePtr frame_msg) {
  RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

  // TODO:
  // 1. transform to the right-hand rule coordinate system
  // 2. Publish the frame marker
}
} // namespace visualization_nodes
} // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(
    vslam_components::visualization_nodes::RvizVisualNode)