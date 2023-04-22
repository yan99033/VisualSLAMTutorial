#ifndef VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_
#define VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_

#include "vslam_plugins_base/place_recognition.hpp"

namespace vslam_place_recognition_plugins {
  class BagOfWords : public vslam_place_recognition_base::PlaceRecognition {
  public:
    void initialize(int top_k = 1) override;

    // Add the visual features of the current frame to the database for future retrieval
    void add_to_database(long unsigned int kf_id, const cv::Mat& visual_features) override;

    // Given the visual features, find the top_k matches
    std::vector<Result> query(const cv::Mat& visual_features) override;
  };
}  // namespace vslam_place_recognition_plugins

#endif
