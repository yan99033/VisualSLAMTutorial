#include "vslam_utils/service_client.hpp"

namespace vslam_utils {

  template <class ServiceType>
  ServiceClient<ServiceType>::ServiceClient(const std::string& service_name,
                                            const rclcpp::Node::SharedPtr& provided_node)
      : service_name_(service_name), node_(provided_node) {
    callback_group_
        = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    client_ = node_->create_client<ServiceType>(service_name, rclcpp::SystemDefaultsQoS(),
                                                callback_group_);
  }

  template <class ServiceType>
  typename ServiceClient<ServiceType>::ResponseType::SharedPtr ServiceClient<ServiceType>::invoke(
      typename ServiceClient<ServiceType>::RequestType::SharedPtr& request,
      const std::chrono::nanoseconds timeout) {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(service_name_
                                 + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(node_->get_logger(), "%s service client: waiting for service to appear...",
                  service_name_.c_str());
    }

    RCLCPP_DEBUG(node_->get_logger(), "%s service client: send async request",
                 service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_.spin_until_future_complete(future_result, timeout)
        != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  template <class ServiceType>
  bool ServiceClient<ServiceType>::wait_for_service(const std::chrono::nanoseconds timeout) {
    return client_->wait_for_service(timeout);
  }

}  // namespace vslam_utils