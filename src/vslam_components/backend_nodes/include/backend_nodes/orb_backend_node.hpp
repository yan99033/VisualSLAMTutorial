// TODO: Licence

#ifndef BACKEND_NODES__ORB_BACKEND_NODE_HPP_
#define BACKEND_NODES__ORB_BACKEND_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/state.hpp"
#include "vslam_srvs/srv/get_state.hpp"

namespace vslam_components {
  namespace backend_nodes {
    class OrbBackend : public rclcpp::Node {
    public:
      explicit OrbBackend(const rclcpp::NodeOptions& options);

    private:
      void get_state_callback(const std::shared_ptr<vslam_srvs::srv::GetState::Request> request,
                              const std::shared_ptr<vslam_srvs::srv::GetState::Response> response);

      rclcpp::Service<vslam_srvs::srv::GetState>::SharedPtr get_state_srv_;

      uint8_t state_{vslam_msgs::msg::State::INITIALIZATION};
    };
  }  // namespace backend_nodes
}  // namespace vslam_components

#endif  // BACKEND_NODES__ORB_BACKEND_NODE_HPP_