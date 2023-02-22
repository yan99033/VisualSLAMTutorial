#include "feature_matching_nodes/orb_matcher_node.hpp"

#include <chrono>

#include "vslam_msgs/msg/state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using GetStateResponseFuture = rclcpp::Client<vslam_srvs::srv::GetState>::SharedFuture;

namespace vslam_components {
  namespace feature_matching_nodes {

    OrbMatcherNode::OrbMatcherNode(const rclcpp::NodeOptions& options)
        : Node("orb_matcher_node", options) {
      get_state_request_ = std::make_shared<vslam_srvs::srv::GetState::Request>();
      get_state_client_ = create_client<vslam_srvs::srv::GetState>("get_state");

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&OrbMatcherNode::frame_matching_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    void OrbMatcherNode::frame_matching_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "OrbMatcherNode: unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "OrbMatcherNode: Received frame: %u", frame_msg->id);

      while (!get_state_client_->wait_for_service(0.5s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      // https://github.com/ros2/demos/blob/humble/composition/src/client_component.cpp
      // In order to wait for a response to arrive, we need to spin().
      // However, this function is already being called from within another spin().
      // Unfortunately, the current version of spin() is not recursive and so we
      // cannot call spin() from within another spin().
      // Therefore, we cannot wait for a response to the request we just made here
      // within this callback, because it was executed by some other spin function.
      // The workaround for this is to give the async_send_request() method another
      // argument which is a callback that gets executed once the future is ready.
      // We then return from this callback so that the existing spin() function can
      // continue and our callback will get called once the response is received.
      vslam_msgs::msg::State system_state;
      auto response_received_callback = [&system_state](GetStateResponseFuture future) {
        system_state = future.get()->system_state;
      };
      auto future_result
          = get_state_client_->async_send_request(get_state_request_, response_received_callback);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OrbMatcherNode: State: %u ", system_state.state);
    }

  }  // namespace feature_matching_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::feature_matching_nodes::OrbMatcherNode)