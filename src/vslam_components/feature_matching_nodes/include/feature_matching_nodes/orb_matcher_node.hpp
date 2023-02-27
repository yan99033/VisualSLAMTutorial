#ifndef FEATURE_MATCHING_NODES__ORB_MATCHING_NODE_HPP_
#define FEATURE_MATCHING_NODES__ORB_MATCHING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_srvs/srv/get_state.hpp"
#include "vslam_srvs/srv/set_state.hpp"
#include "vslam_utils/service_client.hpp"

namespace vslam_components {
  namespace feature_matching_nodes {
    class OrbMatcherNode : public rclcpp::Node {
    public:
      explicit OrbMatcherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

      void set_service_clients(
          const vslam_utils::ServiceClient<vslam_srvs::srv::GetState>::SharedPtr get_state_client,
          const vslam_utils::ServiceClient<vslam_srvs::srv::SetState>::SharedPtr set_state_client);

    private:
      void frame_matching_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg);

      vslam_srvs::srv::GetState::Request::SharedPtr get_state_request_;
      vslam_srvs::srv::SetState::Request::SharedPtr set_state_request_;

      bool service_clients_ready{false};

      // Service clients
      vslam_utils::ServiceClient<vslam_srvs::srv::GetState>::SharedPtr get_state_client_;
      vslam_utils::ServiceClient<vslam_srvs::srv::SetState>::SharedPtr set_state_client_;

      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      // for re-publishing the frame message without creating a copy
      std::weak_ptr<std::remove_pointer<decltype(frame_pub_.get())>::type> captured_frame_pub_;
    };

  }  // namespace feature_matching_nodes
}  // namespace vslam_components

#endif  // FEATURE_MATCHING_NODES__ORB_MATCHING_NODE_HPP_