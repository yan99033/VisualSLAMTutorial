#include "vslam_place_recognition_plugins/bag_of_words.hpp"

namespace vslam_place_recognition_plugins {

  BagOfWords::~BagOfWords() {
    database_->clear();
    keyframe_index_pairs_.clear();
    std::cerr << "Terminated BagOfWords" << std::endl;
  }

  void BagOfWords::initialize(const std::string& input, const int top_k, const double score_thresh,
                              const int ignore_last_n_keyframes) {
    top_k_ = top_k;

    ignore_last_n_keyframes_ = ignore_last_n_keyframes;

    score_thresh_ = score_thresh;

    vocab_ = DBoW3::Vocabulary(input);
    database_ = std::make_unique<DBoW3::Database>(vocab_, false, 0);
  }

  // Add the visual features of the current frame to the database for future retrieval
  void BagOfWords::add_to_database(long unsigned int kf_id, const cv::Mat& visual_features) {
    DBoW3::EntryId entry_id = database_->add(visual_features);

    keyframe_index_pairs_[entry_id] = kf_id;

    last_entry_id_ = entry_id;
  }

  // Given the visual features, find the top_k matches
  std::vector<BagOfWords::Result> BagOfWords::query(const cv::Mat& visual_features) {
    DBoW3::QueryResults database_results;

    const int max_id = [this] {
      if (ignore_last_n_keyframes_ < 0) {
        return ignore_last_n_keyframes_;
      } else {
        return std::max(0, static_cast<int>(last_entry_id_) - ignore_last_n_keyframes_);
      }
    }();

    database_->query(visual_features, database_results, top_k_, max_id);

    std::vector<Result> results;
    for (const auto& res : database_results) {
      if (res.Score > score_thresh_) {
        results.emplace_back(Result{keyframe_index_pairs_.at(res.Id), res.Score});
      }
    }

    return results;
  }

}  // namespace vslam_place_recognition_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_place_recognition_plugins::BagOfWords, vslam_place_recognition::base::PlaceRecognition)