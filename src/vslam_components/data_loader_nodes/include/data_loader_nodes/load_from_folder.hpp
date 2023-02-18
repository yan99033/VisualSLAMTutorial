// TODO: Licence

#ifndef VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_
#define VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_

#include <string>

// #include "data_loader_nodes/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {

  namespace data_loader_nodes {

    class LoadFromFolder : public rclcpp::Node {
    public:
      explicit LoadFromFolder(const rclcpp::NodeOptions& options);

    protected:
      void on_timer();

    private:
      size_t count_;

      sensor_msgs::msg::CameraInfo cam_info_msg_;

      std::vector<std::string> files_;  //!< Files in a folder
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;

      sensor_msgs::msg::CameraInfo load_camera_info();
    };

  }  // namespace data_loader_nodes

}  // namespace vslam_components

#endif  // VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_