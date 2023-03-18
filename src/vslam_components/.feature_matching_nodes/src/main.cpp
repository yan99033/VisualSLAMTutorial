#include <string>

#include "feature_matching_nodes/orb_matcher_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vslam_srvs/srv/get_state.hpp"
#include "vslam_srvs/srv/set_state.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto orb_matcher_node
      = std::make_shared<vslam_components::feature_matching_nodes::OrbMatcherNode>();

  // Create service clients
  std::string get_state_service_name = "get_state";
  std::string set_state_service_name = "set_state";
  std::string get_keyframe_service_name = "get_keyframe";
  std::string set_keyframe_service_name = "set_keyframe";
  auto get_state_client = std::make_shared<vslam_utils::ServiceClient<vslam_srvs::srv::GetState>>(
      get_state_service_name, orb_matcher_node);
  auto set_state_client = std::make_shared<vslam_utils::ServiceClient<vslam_srvs::srv::SetState>>(
      set_state_service_name, orb_matcher_node);
  auto get_keyframe_client
      = std::make_shared<vslam_utils::ServiceClient<vslam_srvs::srv::GetKeyframe>>(
          get_keyframe_service_name, orb_matcher_node);
  auto set_keyframe_client
      = std::make_shared<vslam_utils::ServiceClient<vslam_srvs::srv::SetKeyframe>>(
          set_keyframe_service_name, orb_matcher_node);

  orb_matcher_node->set_service_clients(get_state_client, set_state_client, get_keyframe_client,
                                        set_keyframe_client);

  rclcpp::spin(orb_matcher_node);
  rclcpp::shutdown();
  return 0;
}