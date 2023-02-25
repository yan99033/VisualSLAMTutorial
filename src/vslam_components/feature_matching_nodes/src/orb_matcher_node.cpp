#include "feature_matching_nodes/orb_matcher_node.hpp"

#include <chrono>

#include "vslam_msgs/msg/state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using GetStateResponseFuture = rclcpp::Client<vslam_srvs::srv::GetState>::SharedFuture;
using SetStateResponseFuture = rclcpp::Client<vslam_srvs::srv::SetState>::SharedFuture;

namespace vslam_components {
  namespace feature_matching_nodes {

    OrbMatcherNode::OrbMatcherNode(const rclcpp::NodeOptions& options)
        : Node("orb_matcher_node", options) {
      get_state_client_ = create_client<vslam_srvs::srv::GetState>("get_state");

      set_state_client_ = create_client<vslam_srvs::srv::SetState>("set_state");

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

      auto get_state_request_ = std::make_shared<vslam_srvs::srv::GetState::Request>();
      auto set_state_request_ = std::make_shared<vslam_srvs::srv::SetState::Request>();

      RCLCPP_INFO(this->get_logger(), "OrbMatcherNode: Received frame: %u", frame_msg->id);

      while (!get_state_client_->wait_for_service(0.5s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the get_state service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "get_state service not available, waiting again...");
      }

      // Since we are not able to get the state in this callback, we have to create a service
      // client wrapper to get the system's state. Then, we can create another callback
      // to obtain the response. It must have its own spin()!
      // see nav2_util::ServiceClient

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
      auto get_state_callback = [this](GetStateResponseFuture future) {
        // Check the lambda function! not returning the value (context matters)
        this->system_state_.state = future.get()->system_state.state;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "1: State: %u ", this->system_state_.state);
      };
      get_state_client_->async_send_request(get_state_request_, get_state_callback);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "2: State: %u ", this->system_state_.state);

      if (this->system_state_.state == vslam_msgs::msg::State::INITIALIZATION) {
        set_state_request_->system_state.state = vslam_msgs::msg::State::ATTEMPT_INITIALIZATION;
      } else {
        set_state_request_->system_state.state = vslam_msgs::msg::State::INITIALIZATION;
      }

      while (!set_state_client_->wait_for_service(0.5s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the set_state service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "set_state service not available, waiting again...");
      }

      auto set_state_callback = [](SetStateResponseFuture future) {
        bool success = future.get()->success;
        if (!success) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the set_state service. Exiting.");
          return;
        }
      };
      set_state_client_->async_send_request(set_state_request_, set_state_callback);
    }

  }  // namespace feature_matching_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::feature_matching_nodes::OrbMatcherNode)