#include "feature_matching_nodes/orb_matcher_node.hpp"

#include <chrono>

#include "vslam_msgs/msg/state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace vslam_components {
  namespace feature_matching_nodes {

    OrbMatcherNode::OrbMatcherNode(const rclcpp::NodeOptions& options)
        : Node("orb_matcher_node", options) {
      get_state_request_ = std::make_shared<vslam_srvs::srv::GetState::Request>();
      set_state_request_ = std::make_shared<vslam_srvs::srv::SetState::Request>();
      get_keyframe_request_ = std::make_shared<vslam_srvs::srv::GetKeyframe::Request>();
      set_keyframe_request_ = std::make_shared<vslam_srvs::srv::SetKeyframe::Request>();

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>(
          "in_frame", 10, std::bind(&OrbMatcherNode::frame_matching_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    void OrbMatcherNode::set_service_clients(
        const vslam_utils::ServiceClient<vslam_srvs::srv::GetState>::SharedPtr get_state_client,
        const vslam_utils::ServiceClient<vslam_srvs::srv::SetState>::SharedPtr set_state_client,
        const vslam_utils::ServiceClient<vslam_srvs::srv::GetKeyframe>::SharedPtr
            get_keyframe_client,
        const vslam_utils::ServiceClient<vslam_srvs::srv::SetKeyframe>::SharedPtr
            set_keyframe_client) {
      get_state_client_ = get_state_client;
      set_state_client_ = set_state_client;

      get_keyframe_client_ = get_keyframe_client;
      set_keyframe_client_ = set_keyframe_client;

      service_clients_ready = true;
    }

    void OrbMatcherNode::frame_matching_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) {
      if (!service_clients_ready) {
        RCLCPP_INFO(this->get_logger(), "Service clients are not ready!\n");
        return;
      }

      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "Unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Received frame: %u", frame_msg->id);

      auto get_state_response
          = get_state_client_->invoke(get_state_request_, std::chrono::milliseconds(10));

      RCLCPP_INFO(this->get_logger(), "State: %u ", get_state_response->system_state.state);

      // if the state is tracking
      //   get the keyframe from backend
      //   match features between the current frame and the keyframe
      //   set the current frame as the keyframe
      // else
      //   Set the current frame as the keyframe
      //   set the state to tracking
      if (get_state_response->system_state.state == vslam_msgs::msg::State::TRACKING) {
        get_keyframe_request_->frame_id = -1;

        auto get_keyframe_response
            = get_keyframe_client_->invoke(get_keyframe_request_, std::chrono::milliseconds(10));

        auto keyframe = get_keyframe_response->keyframe;

        RCLCPP_INFO(this->get_logger(), "Tracking between frame %d <-> %d", keyframe.id,
                    frame_msg->id);

      } else {
        set_state_request_->system_state.state = vslam_msgs::msg::State::TRACKING;
        set_state_client_->invoke(set_state_request_, std::chrono::milliseconds(10));
      }

      set_keyframe_request_->keyframe = *frame_msg;

      auto set_keyframe_response
          = set_keyframe_client_->invoke(set_keyframe_request_, std::chrono::milliseconds(10));
      if (!set_keyframe_response->success) {
        throw std::runtime_error("OrbMatcherNode: failed to set the frame as keyframe");
      }

      RCLCPP_INFO(this->get_logger(), "Set frame %d as keyframe", frame_msg->id);
    }

  }  // namespace feature_matching_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::feature_matching_nodes::OrbMatcherNode)