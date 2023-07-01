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

  void Map::cleanUpStaleKeyframesMappoints(const long unsigned int min_kf_id, const long unsigned int max_kf_id) {
    if (cleaning_stale_keyframes_mappoints_) {
      return;
    }

    cleaning_stale_keyframes_mappoints_ = true;

    std::vector<vslam_datastructure::Frame::SharedPtr> keyframes_to_remove;

    vslam_datastructure::Frame::SharedPtr prev_kf_ptr{nullptr};
    std::optional<double> prev_rel_translation;
    int removed_mps{0};
    int removed_kfs{0};
    for (auto& kf_ptr : keyframes_) {
      if (!kf_ptr.get() || kf_ptr->isBad() || kf_ptr->id() < min_kf_id) {
        continue;
      }

      // If the keyframe is being optimized or used for camera tracking or larger than the max id, stop processing the
      // rest
      if (kf_ptr->id() > max_kf_id || kf_ptr->active_tracking_state || kf_ptr->active_ba_state) {
        break;
      }

      std::set<vslam_datastructure::Frame*> projected_keyframes;

      for (const auto& pt : kf_ptr->points()) {
        if (pt->hasMappoint() && pt->isMappointHost()) {
          // if there are less than two projections, remove the map point
          if (pt->mappoint()->projections().size() < 2) {
            pt->deleteMappoint();
            removed_mps++;
            continue;
          }

          for (const auto other_pt : pt->mappoint()->projections()) {
            assert(other_pt->frame());

            if (other_pt->frame()->isBad()) {
              continue;
            }

            projected_keyframes.insert(other_pt->frame());
          }
        }
      }

      // If the map points weren't projected on more than two frames, the keyframe is an outlier keyframe
      if (projected_keyframes.size() < 2 && (!kf_ptr->active_tracking_state)) {
        kf_ptr->setBad();
        removed_kfs++;
      }
    }

    cleaning_stale_keyframes_mappoints_ = false;
  }

}  // namespace vslam_datastructure
