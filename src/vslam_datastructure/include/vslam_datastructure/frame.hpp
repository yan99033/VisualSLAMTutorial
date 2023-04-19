#ifndef VSLAM_DATASTRUCTURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_HPP_

#include <memory>
#include <opencv2/core/mat.hpp>

#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  // Forward declaration
  struct Point;
  class MapPoint;
  using Points = std::vector<std::shared_ptr<Point>>;
  using MapPoints = std::vector<std::shared_ptr<MapPoint>>;

  class Frame {
  public:
    using SharedPtr = std::shared_ptr<Frame>;
    using WeakPtr = std::weak_ptr<Frame>;

    Frame() = delete;

    Frame(const cv::Mat& K);

    cv::Mat get_image() const;

    // Get the camera pose
    cv::Mat T_f_w();

    // Set the camera pose
    void set_pose(const cv::Mat& T_f_w);

    // Set the id, timestamp, image
    void from_msg(vslam_msgs::msg::Frame* frame_msg);

    // Set the id, timestamp, image, pose and points
    // skip_loaded flag to skip the loaded data (id, timestamp and image)
    // no_points flag to skip the 2d keypoints and 3d map points
    void to_msg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded = false, const bool no_mappoints = false);

    // Set the points
    void set_points(Points& points);

    // Get points
    inline const Points& get_points() const { return points_; }

    void set_map_points(MapPoints& mappoints, const std::vector<size_t> indices);

    // Get the numbe of map points
    size_t get_num_mps();

    // Check if there are points available to match or map
    inline bool has_points() const { return !points_.empty(); }

    // Get the frame id
    inline long unsigned int id() const { return id_; }

    // Set the frame as keyframe
    void set_keyframe();

    // A boolean to indicate if the frame is a keyframe
    inline bool is_keyframe() const { return is_keyframe_; }

    // Set pose constraints between adjacent keyframes
    void set_T_this_prev_kf(Frame::SharedPtr prev_kf, const cv::Mat& T_this_prev);
    void add_T_this_next_kf(Frame::SharedPtr next_kf, const cv::Mat& T_this_next);

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
    cv::Mat K_;                                  //!< 3x3 camera matrix
    bool is_keyframe_{false};                    //!< a boolean to indicate if the frame is a keyframe
    std::mutex data_mutex_;                      //!< Mutex for synchronizing reading and writing frame data
    static constexpr double max_reproj_err_{
        8.0};  //!< max reprojection error in pixel to associate a map point with a keypoint

    // Constraints between current (key)frame and the adjacent keyframes
    // A keyframe can only have a parent but can have a number of children (e.g., from loop-closure detection)
    std::pair<Frame::WeakPtr, cv::Mat> T_this_prev_kf_;
    std::vector<std::pair<Frame::WeakPtr, cv::Mat>> T_this_next_kf_;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
