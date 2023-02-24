#include "backend_nodes/orb_backend_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace vslam_components {
  namespace backend_nodes {
    OrbBackendNode::OrbBackendNode(const rclcpp::NodeOptions& options)
        : Node("orb_backend_node", options) {
      get_state_srv_ = create_service<vslam_srvs::srv::GetState>(
          "get_state", std::bind(&OrbBackendNode::get_state_callback, this, _1, _2));
      set_state_srv_ = create_service<vslam_srvs::srv::SetState>(
          "set_state", std::bind(&OrbBackendNode::set_state_callback, this, _1, _2));
    }

    void OrbBackendNode::get_state_callback(
        const std::shared_ptr<vslam_srvs::srv::GetState::Request> request,
        const std::shared_ptr<vslam_srvs::srv::GetState::Response> response) {
      (void)request;
      RCLCPP_INFO(this->get_logger(), "Requested a GetState service");
      response->system_state.state = state_;
    }

    void OrbBackendNode::set_state_callback(
        const std::shared_ptr<vslam_srvs::srv::SetState::Request> request,
        const std::shared_ptr<vslam_srvs::srv::SetState::Response> response) {
      state_ = request->system_state.state;

      RCLCPP_INFO(this->get_logger(), "Requested a SetState service");
      response->success = true;
    }

  }  // namespace backend_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::backend_nodes::OrbBackendNode)