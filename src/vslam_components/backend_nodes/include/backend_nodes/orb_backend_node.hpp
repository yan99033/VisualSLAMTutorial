// TODO: Licence

#ifndef BACKEND_NODES__ORB_BACKEND_NODE_HPP_
#define BACKEND_NODES__ORB_BACKEND_NODE_HPP_

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/state.hpp"
#include "vslam_srvs/srv/get_keyframe.hpp"
#include "vslam_srvs/srv/get_state.hpp"
#include "vslam_srvs/srv/set_keyframe.hpp"
#include "vslam_srvs/srv/set_state.hpp"

namespace vslam_components {
  namespace backend_nodes {
    class OrbBackendNode : public rclcpp::Node {
    public:
      explicit OrbBackendNode(const rclcpp::NodeOptions& options);

    private:
      void get_state_callback(const std::shared_ptr<vslam_srvs::srv::GetState::Request> request,
                              const std::shared_ptr<vslam_srvs::srv::GetState::Response> response);

      void set_state_callback(const std::shared_ptr<vslam_srvs::srv::SetState::Request> request,
                              const std::shared_ptr<vslam_srvs::srv::SetState::Response> response);

      void get_keyframe_callback(
          const std::shared_ptr<vslam_srvs::srv::GetKeyframe::Request> request,
          const std::shared_ptr<vslam_srvs::srv::GetKeyframe::Response> response);

      void set_keyframe_callback(
          const std::shared_ptr<vslam_srvs::srv::SetKeyframe::Request> request,
          const std::shared_ptr<vslam_srvs::srv::SetKeyframe::Response> response);

      rclcpp::Service<vslam_srvs::srv::GetState>::SharedPtr get_state_srv_;
      rclcpp::Service<vslam_srvs::srv::SetState>::SharedPtr set_state_srv_;

      rclcpp::Service<vslam_srvs::srv::GetKeyframe>::SharedPtr get_keyframe_srv_;
      rclcpp::Service<vslam_srvs::srv::SetKeyframe>::SharedPtr set_keyframe_srv_;

      uint8_t state_{vslam_msgs::msg::State::INITIALIZATION};

      vslam_msgs::msg::Frame::SharedPtr current_keyframe_;
      std::map<int, vslam_msgs::msg::Frame::SharedPtr> keyframes_;
    };
  }  // namespace backend_nodes
}  // namespace vslam_components

#endif  // BACKEND_NODES__ORB_BACKEND_NODE_HPP_