#include "vslam_place_recognition_plugins/bag_of_words.hpp"

namespace vslam_place_recognition_plugins {
  void BagOfWords::initialize(const int top_k, const std::string& input) {
    top_k_ = top_k;

    vocab_ = DBoW3::Vocabulary(input);
    database_ = std::make_unique<DBoW3::Database>(vocab_, false, 0);
  }

  // Add the visual features of the current frame to the database for future retrieval
  void BagOfWords::add_to_database(long unsigned int kf_id, const cv::Mat& visual_features) {}

  // Given the visual features, find the top_k matches
  std::vector<BagOfWords::Result> BagOfWords::query(const cv::Mat& visual_features) { return {}; }

}  // namespace vslam_place_recognition_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_place_recognition_plugins::BagOfWords, vslam_place_recognition_base::PlaceRecognition)