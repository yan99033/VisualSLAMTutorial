#ifndef VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_
#define VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_

#include <DBoW3/DBoW3.h>

#include "vslam_plugins_base/place_recognition.hpp"

namespace vslam_place_recognition_plugins {
  class BagOfWords : public virtual vslam_place_recognition::base::PlaceRecognition {
  public:
    ~BagOfWords();

    void initialize(const std::string& input = std::string(), const int top_k = 1, const double score_thresh = 0.9,
                    const int ignore_last_n_keyframes = -1) override;

    // Add the visual features of the current frame to the database for future retrieval
    void add_to_database(long unsigned int kf_id, const cv::Mat& visual_features) override;

    // Given the visual features, find the top_k matches
    std::vector<Result> query(const cv::Mat& visual_features) override;

    inline std::string get_plugin_name() override { return "vslam_place_recognition_plugins::BagOfWords"; }

  private:
    DBoW3::Vocabulary vocab_;
    std::unique_ptr<DBoW3::Database> database_;
    std::unordered_map<DBoW3::EntryId, long unsigned int> keyframe_index_pairs_;

    DBoW3::EntryId last_entry_id_{0};

    int top_k_{1};

    // Filter the results within the last n keyframes
    // -1 means no filtering is performed
    int ignore_last_n_keyframes_{-1};

    double score_thresh_{0.9};
  };
}  // namespace vslam_place_recognition_plugins

#endif
