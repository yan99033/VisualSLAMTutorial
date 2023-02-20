#include "backend_nodes/orb_backend_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace vslam_components {
  namespace backend_nodes {
    OrbBackend::OrbBackend(const rclcpp::NodeOptions& options) : Node("orb_backend_node", options) {
      get_state_srv_ = create_service<vslam_srvs::srv::GetState>(
          "get_state", std::bind(&OrbBackend::get_state_callback, this, _1, _2));
    }

    void OrbBackend::get_state_callback(
        const std::shared_ptr<vslam_srvs::srv::GetState::Request> request,
        const std::shared_ptr<vslam_srvs::srv::GetState::Response> response) {
      RCLCPP_INFO(this->get_logger(), "OrbBackend: requested a GetState service");
      response->system_state.state = state_;
    }

  }  // namespace backend_nodes
}  // namespace vslam_components