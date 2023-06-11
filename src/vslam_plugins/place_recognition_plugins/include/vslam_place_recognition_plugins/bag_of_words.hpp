#ifndef VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_
#define VSLAM_PLACE_RECOGNITION_PLUGINS__ORB_HPP_

#include <DBoW3/DBoW3.h>

#include "vslam_plugins_base/place_recognition.hpp"

namespace vslam_place_recognition_plugins {
  class BagOfWords : public virtual vslam_place_recognition::base::PlaceRecognition {
  public:
    ~BagOfWords();

    /// BagOfWords place recognition intializer
    /**
     * \param[in] input path to the vocabulary file
     * \param[in] top_k return the top k results
     * \param[in] score_thresh similarity scores larger than this are considered a good match
     * \param[in] ignore_last_n_keyframes ignore the matches within the last n keyframes. -1 means no filtering
     */
    void initialize(const std::string& input = std::string(), const int top_k = 1, const double score_thresh = 0.9,
                    const int ignore_last_n_keyframes = -1) override;

    /// Add the visual features of the current frame to the database for future retrieval
    /**
     * \param[in] kf_id the keyframe id associated with the visual features
     * \param[in] visual_features the visual features of the keyframe
     */
    void addToDatabase(long unsigned int kf_id, const cv::Mat& visual_features) override;

    /// Given the visual features, find the top_k matches
    /**
     * \param[in] visual_features the visual feature of the keyfram
     * \return results containing the keyframe ids and their score
     */
    std::vector<Result> query(const cv::Mat& visual_features) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_place_recognition_plugins::BagOfWords"; }

  private:
    /// DBoW3 vocabulary
    DBoW3::Vocabulary vocab_;

    /// DBoW3 database
    std::unique_ptr<DBoW3::Database> database_;

    /// Map between the DBoW3 entry and keyframe ids
    std::unordered_map<DBoW3::EntryId, long unsigned int> keyframe_index_pairs_;

    /// Last DBoW3 entry id
    DBoW3::EntryId last_entry_id_{0};

    /// Returns the top k results
    int top_k_{1};

    /// Filter the results within the last n keyframes. -1 means no filtering
    int ignore_last_n_keyframes_{-1};

    /// Consider match scores below this a mismatch
    double score_thresh_{0.9};
  };
}  // namespace vslam_place_recognition_plugins

#endif
