#ifndef VSLAM_UTILS__SERVICE_CLIENT_HPP__
#define VSLAM_UTILS__SERVICE_CLIENT_HPP__

#include "rclcpp/rclcpp.hpp"

namespace vslam_utils {

  template <class ServiceType> class ServiceClient {
  public:
    /**
     * @brief A constructor
     * @param service_name name of the service to call
     * @param node Node to create the service client off of
     */
    explicit ServiceClient(const std::string& service_name, const rclcpp::Node::SharedPtr& node);

    using RequestType = typename ServiceType::Request;
    using ResponseType = typename ServiceType::Response;

    using SharedPtr = std::shared_ptr<ServiceClient<ServiceType>>;

    /**
     * @brief Invoke the service and block until completed or timed out
     * @param request The request object to call the service using
     * @param timeout Maximum timeout to wait for, default infinite
     * @return Response A pointer to the service response from the request
     */
    typename ResponseType::SharedPtr invoke(typename RequestType::SharedPtr& request,
                                            const std::chrono::nanoseconds timeout
                                            = std::chrono::nanoseconds(-1));

    /**
     * @brief Block until a service is available or timeout
     * @param timeout Maximum timeout to wait for, default infinite
     * @return bool true if service is available
     */
    bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max());

  protected:
    std::string service_name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    typename rclcpp::Client<ServiceType>::SharedPtr client_;
  };
}  // namespace vslam_utils
#endif  // VSLAM_UTILS__SERVICE_CLIENT_HPP__