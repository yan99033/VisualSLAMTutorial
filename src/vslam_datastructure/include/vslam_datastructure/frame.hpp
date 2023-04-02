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
    cv::Mat getPose() const;
    cv::Mat getImage() const;

    // Set the id, timestamp, image
    void fromMsg(vslam_msgs::msg::Frame* frame_msg);

    //
    vslam_msgs::msg::Frame toMsg() const;

    // Set the camera pose
    void setPose(const cv::Mat& T_f_w);

    // Set the points
    void setPoints(Points& points);

    // Get points
    Points* getPoints();

    // Check if there are points available to match or map
    bool hasPoints() const;

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
