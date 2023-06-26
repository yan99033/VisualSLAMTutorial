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

#include "vslam_datastructure/map.hpp"

#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  Map::~Map() { clear(); }

  void Map::addKeyframe(Frame::SharedPtr keyframe) {
    assert(keyframe->isKeyframe());

    std::lock_guard<std::mutex> lck(map_mutex_);
    keyframes_.push_back(keyframe);
    keyframe_map_[keyframe->id()] = keyframe;
  }

  void Map::removeKeyframe(Frame::SharedPtr keyframe) {
    for (auto it = keyframes_.begin(); it != keyframes_.end();) {
      if ((*it)->id() == keyframe->id()) {
        std::lock_guard<std::mutex> lck(map_mutex_);
        keyframes_.erase(it);

        if (keyframe_map_.find(keyframe->id()) != keyframe_map_.end()) {
          keyframe_map_.erase(keyframe->id());
        }

        break;
      } else {
        ++it;
      }
    }
  }

  void Map::clear() {
    std::lock_guard<std::mutex> lck(map_mutex_);
    keyframe_map_.clear();
    keyframes_.clear();
  }

  Frame::SharedPtr Map::getKeyframe(const long unsigned int id) const {
    std::lock_guard<std::mutex> lck(map_mutex_);
    if (keyframe_map_.find(id) == keyframe_map_.end()) {
      return nullptr;
    } else {
      return keyframe_map_.at(id);
    }
  }

  std::pair<CoreKfsSet, CoreMpsSet> Map::getCoreKeyframesMappoints(const size_t num_core_kfs) {
    std::lock_guard<std::mutex> lck(map_mutex_);
    CoreKfsSet core_keyframes;
    CoreMpsSet core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    while (kfs_it != keyframes_.rend() && core_keyframes.size() < num_core_kfs) {
      auto kf = *kfs_it;

      if (!kf.get() || kf->isBad()) {
        kfs_it++;
        continue;
      }

      core_keyframes.insert(kf.get());

      for (auto pt : kf->points()) {
        if (pt->hasMappoint() && pt->mappoint()->projections().size() > 1) {
          core_mappoints.insert(pt->mappoint().get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  std::vector<vslam_msgs::msg::Frame> Map::getAllKeyframeMsgs() const {
    std::vector<vslam_msgs::msg::Frame> keyframe_msgs;
    for (const auto& kf : keyframes_) {
      if (kf->isBad()) {
        continue;
      }

      vslam_msgs::msg::Frame keyframe_msg;
      kf->toMsg(&keyframe_msg, true);
      keyframe_msgs.push_back(keyframe_msg);
    }

    return keyframe_msgs;
  }

  std::list<Frame::SharedPtr> Map::keyframes() {
    std::lock_guard<std::mutex> lck(map_mutex_);

    return std::list<Frame::SharedPtr>(keyframes_);
  }

}  // namespace vslam_datastructure
