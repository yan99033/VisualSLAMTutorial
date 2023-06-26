/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_DATASTRUCTURE__MAP_HPP_
#define VSLAM_DATASTRUCTURE__MAP_HPP_

#include <list>
#include <mutex>
#include <set>
#include <unordered_map>

#include "vslam_datastructure/frame.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  template <typename T> struct Cmp {
    bool operator()(const T* lhs, const T* rhs) const { return lhs->id() < rhs->id(); }
  };

  using CoreKfsSet = std::set<Frame*, Cmp<Frame>>;
  using CoreMpsSet = std::set<MapPoint*, Cmp<MapPoint>>;

  class Map {
  public:
    /// Default constructor
    Map() = default;

    /// Destructor
    ~Map();

    /// Add a new keyframe
    /**
     * \param[in] frame a new keyframe
     */
    void addKeyframe(Frame::SharedPtr keyframe);

    /// Remove a keyframe
    /**
     * \param[in] frame the keyframe to be removed
     */
    void removeKeyframe(Frame::SharedPtr keyframe);

    /// Clear the map
    void clear();

    /// Get a keyframe using the id.
    /**
     * \return keyframe with the id or a nullptr if the keyframe cannot be found
     */
    Frame::SharedPtr getKeyframe(const long unsigned int id) const;

    /// Get the latest `num_core_kfs_` core keyframes and their map points
    /**
     * \return core keyframes and map points
     */
    std::pair<CoreKfsSet, CoreMpsSet> getCoreKeyframesMappoints(const size_t num_core_kfs);

    /// Convert all keyframes to keyframe msgs
    /**
     * \return a vector containing frame messages
     */
    std::vector<vslam_msgs::msg::Frame> getAllKeyframeMsgs() const;

    /// Get a shallow copy of the keyframes
    std::list<Frame::SharedPtr> keyframes();

  private:
    /// kayframe map (where keyframes can be looked up using their id)
    std::unordered_map<long unsigned int, Frame::SharedPtr> keyframe_map_;

    /// All keyframes
    std::list<Frame::SharedPtr> keyframes_;

    /// @brief  mutex for modifying the map
    mutable std::mutex map_mutex_;
  };
}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__MAP_HPP_