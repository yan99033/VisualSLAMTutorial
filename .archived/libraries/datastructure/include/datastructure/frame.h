#ifndef __DATASTRUCTURE_H__
#define __DATASTRUCTURE_H__

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "feature_detector/feature_detector.h"

namespace vslam_libs {
  namespace datastructure {
    class Frame {
    public:
      explicit Frame(const cv::Mat& image);
      Frame(const cv::Mat& image, const cv::Mat& cam_mat,
            feature_detector::OrbFeatureDetector* detector = nullptr);

      // Getters
      cv::Mat getDescriptors() const;
      const std::vector<cv::KeyPoint>& getKeypoints() const;
      void getPose(Sophus::SE3d& Tcw);
      cv::Mat getImage() const;

      // Setter
      void setPose(const Sophus::SE3d& Tcw);

    private:
      // detect and compute feature descriptors
      void detectAndComputeDescriptors();

      cv::Mat image;    //<! the image of the frame
      cv::Mat cam_mat;  //<! 3x3 camera matrix containing the camera intrinsics
      feature_detector::OrbFeatureDetector* detector;  //<! ORB feature detector

      // Keypoints and their feature descciptors computed by OrbFeatureDescriptor
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;

      // Camera pose
      Sophus::SE3d Tcw;

      std::mutex mutex;
    };

    using FramePtr = std::shared_ptr<Frame>;

  }  // namespace datastructure
}  // namespace vslam_libs

#endif  // __DATASTRUCTURE_H__