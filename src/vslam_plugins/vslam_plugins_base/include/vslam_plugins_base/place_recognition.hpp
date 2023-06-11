#ifndef VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_
#define VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

#include "vslam_plugins_base/base.hpp"

namespace vslam_place_recognition {
  namespace base {
    class PlaceRecognition : public virtual vslam_plugin::base::Plugin {
    public:
      /// Data structure for keeping the result
      struct Result {
        /// Keyframe id
        long unsigned int kf_id;

        /// Match score
        double score;
      };

      // The additional input could be a vocabulary database or a trained deep neural network model
      // used for place recognition

      /// BagOfWords place recognition intializer
      /**
       * \param[in] input could be a vocabulary database or a trained deep neural network model used for place
       *  recognition
       * \param[in] top_k return the top k results
       * \param[in] score_thresh similarity scores larger than this are considered a good match
       * \param[in] ignore_last_n_keyframes ignore the matches within the last n keyframes. -1 means no filtering
       */
      virtual void initialize(const std::string& input = std::string(), const int top_k = 1,
                              const double score_thresh = 0.9, const int ignore_last_n_keyframes = -1)
          = 0;

      /// Add the visual features of the current frame to the database for future retrieval
      /**
       * \note cv::Mat to store your visual feature data, as it supports a variety of primitive types
       *
       * \param[in] kf_id the keyframe id associated with the visual features
       * \param[in] visual_features the visual features of the keyframe
       */
      virtual void addToDatabase(long unsigned int kf_id, const cv::Mat& visual_features) = 0;

      /// Given the visual features, find the top_k matches
      /**
       * \param[in] visual_features the visual feature of the keyfram
       * \return results containing the keyframe ids and their score
       */
      virtual std::vector<Result> query(const cv::Mat& visual_features) = 0;

      /// Destructor
      virtual ~PlaceRecognition() {}

    protected:
      /// Constructor
      PlaceRecognition() {}
    };
  }  // namespace base
}  // namespace vslam_place_recognition

#endif  // VSLAM_PLUGINS_BASE__PLACE_RECOGNITION_HPP_