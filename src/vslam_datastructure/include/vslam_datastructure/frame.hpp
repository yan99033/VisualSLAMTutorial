/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_DATASTRUCTURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <unordered_map>

#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  // Forward declaration
  class Point;
  class MapPoint;
  using Points = std::vector<std::shared_ptr<Point>>;
  using MapPoints = std::vector<std::shared_ptr<MapPoint>>;
  using MappointIndexPairs = std::vector<std::pair<size_t, std::shared_ptr<MapPoint>>>;

  class Frame {
  public:
    using SharedPtr = std::shared_ptr<Frame>;
    using KeyframeConstraintsMap = std::unordered_map<const Frame*, cv::Mat>;

    Frame() = default;

    /// Get the 4x4 camera pose
    /**
     * [[r11, r12, r13, tx],
     *  [r21, r22, r23, ty],
     *  [r31, r32, r33, tz],
     *  [  0,   0,   0,  1]]
     *
     * \return camera pose
     */
    cv::Mat T_f_w() const;

    /// Get the 4x4 camera pose (inversed)
    /**
     * [[r11, r12, r13, tx],
     *  [r21, r22, r23, ty],
     *  [r31, r32, r33, tz],
     *  [  0,   0,   0,  1]]
     *
     * \return camera pose
     */
    cv::Mat T_w_f() const;

    /// Get camera matrix
    /**
     * [[fx,  0, cx],
     *  [ 0, fy, cy],
     *  [ 0,  0,  1]]
     *
     * \return Camera matrix
     */
    cv::Mat K() const;

    /// Transform the map points to the this frame and apply sim3 transformation. Then, update the camera pose
    /**
     * \param[in] S_f_w optimized Sim(3) pose from pose graph optimization
     * \param[in] T_f_w scaled SE(3) pose
     */
    void updateSim3PoseAndMps(const cv::Mat& S_f_w, const cv::Mat& T_f_w);

    /// Set the camera pose
    /**
     * \param[in] T_f_w camera pose in the world frame
     */
    void setPose(const cv::Mat& T_f_w);

    /// Convert the frame msg to frame data
    /**
     * Set the id, timestamp, image
     * \param[in] frame_msg Frame message
     */
    void fromMsg(vslam_msgs::msg::Frame* const frame_msg);

    /// Convert the frame data to frame msg
    /**
     * \param[out] frame_msg frame message
     * \param[in] skip_loaded flag to skip the loaded data (id, timestamp and image)
     * \param[in] no_mappoints flag to skip map point data
     */
    // skip_loaded
    // no_points flag to skip the 2d keypoints and 3d map points
    void toMsg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded = false, const bool no_mappoints = false);

    /// Set the points
    /**
     * \param[in] points a vector of points
     */
    void setPoints(Points& points);

    /// Get points
    /**
     * \return a vector of points
     */
    inline const Points& points() const { return points_; }

    /// Get map points of the indices
    /**
     * \param[in] indices of the points containing the map points
     * \return a vector of map points
     */
    MapPoints mappoints(const std::vector<size_t> point_indices);

    /// Set map points to the existing points
    /**
     * Use the set_host flag to make this frame as the host keyframe to the map points
     * \param[in] mappoints map points to be assigned to the points
     * \param[in] point_indices the indices of the points that are associated with the map points
     * \param[in] set_host set true if the map points belong to this frame
     */
    void setMappoints(const MapPoints& mappoints, const std::vector<size_t> point_indices, const bool set_host = false);

    /// Fuse the old map points with the provided new map points
    /**
     * \param[in] mappoint_index_pairs the indices of the points and the new map points to replace the old ones
     */
    void fuseMappoints(const MappointIndexPairs& mappoint_index_pairs);

    /// Check if there are points available to match or map
    inline bool hasPoints() const { return !points_.empty(); }

    /// Get the frame id
    /**
     * \return the frame id
     */
    inline long unsigned int id() const { return id_; }

    /// Set the frame as keyframe
    void setKeyframe();

    /// A boolean to indicate if the frame is a keyframe
    /**
     * \return true if this frame is a keyframe
     */
    inline bool isKeyframe() const { return is_keyframe_; }

    /// Set the parent keyframe of this frame
    /**
     * \param frame[in] the pointer to the parent keyframe
     */
    void setParentKeyframe(Frame* const frame);

    /// Set a pose constraint to a close keyframe
    /**
     * \param[in] next_kf the other keyframe
     * \param[in] T_this_next the relative transformation between this and the other keyframe
     */
    void addTThisOtherKf(const Frame* const next_kf, const cv::Mat& T_this_next);

    /// Get pose constraints
    /**
     * \return the pose constraints between this and other keyframes
     */
    inline const KeyframeConstraintsMap& TThisOtherKfs() const { return T_this_other_kfs_; }

    /// Transform a map point in the world coordinate frame to this camera's coordinate frame
    /**
     * \param[in] world_pos the position of a map point in the world coordinate frame
     * \return the position of the map point in the camera's coordinate frame
     */
    cv::Point3d mappointWorldToCam(const cv::Point3d& world_pos) const;

    /**
     * The active state denotes the frame and its map points must not be removed.
     * Should be applied to the frames and map points used for camera tracking
     */
    std::atomic_bool active_tracking_state{false};

    /**
     * The active state denotes the frame and its map points must not be removed.
     * Should be applied to the frames and map points used for the local BA optimization
     */
    std::atomic_bool active_ba_state{false};

  private:
    /// Iterate through the map points and set the projection constraints
    void setMappointProjections();

    /// frame id
    long unsigned int id_{0};

    /// Timestamp
    double timestamp_{0};

    /// Seconds of ROS2 timestamp
    int ros_timestamp_sec_{0};

    /// Nanoseconds of ROS2 timestamp
    unsigned int ros_timestamp_nanosec_{0};

    /// Camera pose
    /**
     * [[r11, r12, r13, tx],
     *  [r21, r22, r23, ty],
     *  [r31, r32, r33, tz],
     *  [  0,   0,   0,  1]]
     */
    cv::Mat T_f_w_{cv::Mat::eye(4, 4, CV_64F)};

    /// Image of the frame
    cv::Mat image_;

    /// Points in the frame
    /**
     * Each point contains a 2D position and a local feature and may be associated with a map point
     */
    Points points_;

    /// 3x3 camera matrix
    /**
     * [[fx,  0, cx],
     *  [ 0, fy, cy],
     *  [ 0,  0,  1]]
     */
    cv::Mat K_;

    /// A boolean to indicate if the frame is a keyframe
    bool is_keyframe_{false};

    /// Mutex for synchronizing reading and writing frame data
    mutable std::mutex data_mutex_;

    ///  max reprojection error in pixel to associate a map point with a keypoint
    static constexpr double max_reproj_err_{8.0};

    /// The parent keyframe
    Frame* parent_;

    /// Constraints between current (key)frame and the adjacent keyframes
    /**
     * A keyframe can only have a parent but can have a number of children (e.g., from loop-closure detection)
     */
    KeyframeConstraintsMap T_this_other_kfs_;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
