#include "vslam_utils/service_client.hpp"

namespace vslam_utils {

  template <class ServiceType>
  ServiceClientNode<ServiceType>::ServiceClientNode(const rclcpp::NodeOptions& options)
      : Node("service_client_node", options) {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());
    client_
        = create_client<ServiceType>(service_name, rclcpp::SystemDefaultsQoS(), callback_group_);
  }

}  // namespace vslam_utils