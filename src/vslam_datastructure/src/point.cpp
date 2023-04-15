#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  long unsigned int Point::point_count = 0;

  long unsigned int MapPoint::point_count = 0;

  std::vector<size_t> Matches::get_first_indices() const {
    std::vector<size_t> indices;
    for (const auto& [idx, _] : matched_index_pairs) {
      indices.push_back(idx);
    }
    return indices;
  }

  std::vector<size_t> Matches::get_second_indices() const {
    std::vector<size_t> indices;
    for (const auto& [_, idx] : matched_index_pairs) {
      indices.push_back(idx);
    }
    return indices;
  }
}  // namespace vslam_datastructure