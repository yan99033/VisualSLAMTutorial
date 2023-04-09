#ifndef VSLAM_DATASTRUCTURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_HPP_

#include <memory>
#include <opencv2/core/mat.hpp>

#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  // Forward declaration
  struct Point;
  using Points = std::vector<std::shared_ptr<Point>>;

  class Frame {
  public:
    using SharedPtr = std::shared_ptr<Frame>;
    using WeakPtr = std::weak_ptr<Frame>;

    // Getters
    cv::Mat get_pose() const;
    cv::Mat get_image() const;

    // Set the id, timestamp, image
    void from_msg(vslam_msgs::msg::Frame* frame_msg);

    // Set the id, timestamp, image, pose and points
    // A flag to skip the loaded data (id, timestamp and image)
    void to_msg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded = false) const;

    // Set the camera pose
    void set_pose(const cv::Mat& T_f_w);

    // Set the points
    void set_points(Points& points);

    // Get points
    Points* get_points();

    // Get the numbe of map points
    size_t get_num_mps() const;

    // Check if there are points available to match or map
    bool has_points() const;

  private:
    long unsigned int id_{0};                    //!< frame id
    double timestamp_{0};                        //!< timestamp
    int ros_timestamp_sec_{0};                   //!< seconds component of ROS2 timestamp
    unsigned int ros_timestamp_nanosec_{0};      //!< nanoseconds component of ROS2 timestamp
    cv::Mat T_f_w_{cv::Mat::eye(4, 4, CV_64F)};  //!< Camera pose
    cv::Mat image_;                              //!< the image of the frame
    Points points_;                              //!< vector containing 2D and 3D points

    std::mutex mutex;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
