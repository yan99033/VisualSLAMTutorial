#ifndef VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_
#define VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace vslam_place_recognition_base {
  class PlaceRecognition {
  public:
    struct Result {
      long unsigned int kf_id;
      double score;
    };

    // The additional input coult be a vocabulary database or a trained deep neural network model
    // used for place recognition
    virtual void initialize(const std::string& input = std::string(), const int top_k = 1,
                            const double score_thresh = 0.9)
        = 0;

    // Add the visual features of the current frame to the database for future retrieval
    // Use cv::Mat to store your visual feature data, as it supports a variety of primitive types
    virtual void add_to_database(long unsigned int kf_id, const cv::Mat& visual_features) = 0;

    // Given the visual features, find the top_k matches
    virtual std::vector<Result> query(const cv::Mat& visual_features) = 0;

    virtual ~PlaceRecognition() {}

  protected:
    PlaceRecognition() {}

    int top_k_{1};

    double score_thresh_{0.9};
  };
}  // namespace vslam_place_recognition_base

#endif  // VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_