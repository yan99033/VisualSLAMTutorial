// TODO: Licence

#ifndef DATA_LOADER_NODES__LOAD_FROM_FOLDER_HPP_
#define DATA_LOADER_NODES__LOAD_FROM_FOLDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

#include "data_loader_nodes/camera_info.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_utils/undistorter.hpp"

namespace vslam_components {

  namespace data_loader_nodes {

    class LoadFromFolder : public rclcpp::Node {
    public:
      explicit LoadFromFolder(const rclcpp::NodeOptions& options);

    protected:
      void on_timer();

    private:
      size_t count_;

      std::vector<std::string> files_;  //!< Files in a folder
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;
      rclcpp::TimerBase::SharedPtr timer_;

      // Load the camera matrix and undistortion coefficients
      CameraInfo load_camera_info();

      // Undistorter
      std::unique_ptr<vslam_utils::Undistorter> undistorter_{nullptr};

      sensor_msgs::msg::CameraInfo cam_info_msg_;
    };

  }  // namespace data_loader_nodes

}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__LOAD_FROM_FOLDER_HPP_