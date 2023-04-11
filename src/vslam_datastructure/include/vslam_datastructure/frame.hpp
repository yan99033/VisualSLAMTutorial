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
    cv::Mat get_pose() const { return T_f_w_; };
    cv::Mat get_image() const;

    // Set the id, timestamp, image
    void from_msg(vslam_msgs::msg::Frame* frame_msg);

    // Set the id, timestamp, image, pose and points
    // A flag to skip the loaded data (id, timestamp and image)
    void to_msg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded = false) const;

    // Set the camera pose
    void set_pose(const cv::Mat& T_f_w) { T_f_w_ = T_f_w.clone(); };

    // Set the points
    void set_points(Points& points);

    // Get points
    Points* get_points() { return &points_; }

    // Get the numbe of map points
    size_t get_num_mps() const;

    // Check if there are points available to match or map
    inline bool has_points() const { return !points_.empty(); }

    // Get the frame id
    inline long unsigned int id() const { return id_; }

    // Set the frame as keyframe
    void set_keyframe();

    // A boolean to indicate if the frame is a keyframe
    inline bool is_keyframe() const { return is_keyframe_; }

  private:
    // Iterate through the map points and set the projection constraints
    void set_mappoint_projections();

    long unsigned int id_{0};                    //!< frame id
    double timestamp_{0};                        //!< timestamp
    int ros_timestamp_sec_{0};                   //!< seconds component of ROS2 timestamp
    unsigned int ros_timestamp_nanosec_{0};      //!< nanoseconds component of ROS2 timestamp
    cv::Mat T_f_w_{cv::Mat::eye(4, 4, CV_64F)};  //!< Camera pose
    cv::Mat image_;                              //!< the image of the frame
    Points points_;                              //!< vector containing 2D and 3D points
    bool is_keyframe_{false};                    //!< a boolean to indicate if the frame is a keyframe
    std::mutex mutex;

    // Constraints between current (key)frame and the adjacent keyframes
    std::pair<Frame::WeakPtr, cv::Mat> T_this_next_kf_;
    std::pair<Frame::WeakPtr, cv::Mat> T_this_prev_kf_;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
