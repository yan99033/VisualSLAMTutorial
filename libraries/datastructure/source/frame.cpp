#include "datastructure/frame.h"

namespace vslam_libs {

  namespace datastructure {
    Frame::Frame(const cv::Mat& image) : image(image) {}

    Frame::Frame(const cv::Mat& image, const cv::Mat& cam_mat,
                 feature_detector::OrbFeatureDetector* detector)
        : image(image), cam_mat(cam_mat), detector(detector) {
      detectAndComputeDescriptors();
    }

    // Getter (user has to check if the descriptors and keypoints are valid)
    cv::Mat Frame::getDescriptors() const { return descriptors; }
    const std::vector<cv::KeyPoint>& Frame::getKeypoints() const { return keypoints; }

    void Frame::getPose(cv::Mat& R, cv::Mat& t) const {
      R = this->R.clone();
      t = this->t.clone();
    }

    // Set the camera pose
    void Frame::setPose(const cv::Mat& R, const cv::Mat& t) {
      this->R = R.clone();
      this->t = t.clone();
    }

    void Frame::detectAndComputeDescriptors() {
      // NOTE: raise Exception if detector is not a valid pointer
      detector->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    }

  }  // namespace datastructure
}  // namespace vslam_libs