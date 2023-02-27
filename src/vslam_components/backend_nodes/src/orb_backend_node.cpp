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
      get_keyframe_srv_ = create_service<vslam_srvs::srv::GetKeyframe>(
          "get_keyframe", std::bind(&OrbBackendNode::get_keyframe_callback, this, _1, _2));
      set_keyframe_srv_ = create_service<vslam_srvs::srv::SetKeyframe>(
          "set_keyfrmae", std::bind(&OrbBackendNode::set_keyframe_callback, this, _1, _2));
    }

    void OrbBackendNode::get_state_callback(
        const std::shared_ptr<vslam_srvs::srv::GetState::Request> request,
        const std::shared_ptr<vslam_srvs::srv::GetState::Response> response) {
      (void)request;
      RCLCPP_INFO(this->get_logger(), "Requested a GetState service: (%d)", state_);
      response->system_state.state = state_;
    }

    void OrbBackendNode::set_state_callback(
        const std::shared_ptr<vslam_srvs::srv::SetState::Request> request,
        const std::shared_ptr<vslam_srvs::srv::SetState::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Requested a SetState service, (%d -> %d)", state_,
                  request->system_state.state);

      state_ = request->system_state.state;

      response->success = true;
    }

    void OrbBackendNode::get_keyframe_callback(
        const std::shared_ptr<vslam_srvs::srv::GetKeyframe::Request> request,
        const std::shared_ptr<vslam_srvs::srv::GetKeyframe::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Requested a GetKeyframe service: (%lld)", request->frame_id);

      if (state_ == vslam_msgs::msg::State::INITIALIZATION) {
        throw std::runtime_error(std::string(get_keyframe_srv_->get_service_name())
                                 + " service client: no keyframe is available to get");
      }

      if (request->frame_id == -1) {
        response->keyframe = *current_keyframe_;
        response->has_keyframe = true;
      } else if (auto search = keyframes_.find(request->frame_id); search != keyframes_.end()) {
        response->keyframe = *(search->second);
        response->has_keyframe = true;
      } else {
        throw std::runtime_error(std::string(get_keyframe_srv_->get_service_name())
                                 + " service client: cannot find keyframe "
                                 + std::to_string(request->frame_id));
      }
    }

    void OrbBackendNode::set_keyframe_callback(
        const std::shared_ptr<vslam_srvs::srv::SetKeyframe::Request> request,
        const std::shared_ptr<vslam_srvs::srv::SetKeyframe::Response> response) {
      current_keyframe_ = std::make_shared<vslam_msgs::msg::Frame>(request->keyframe);
      keyframes_[request->keyframe.id]
          = std::make_shared<vslam_msgs::msg::Frame>(request->keyframe);

      response->success = true;
    }
  }  // namespace backend_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::backend_nodes::OrbBackendNode)