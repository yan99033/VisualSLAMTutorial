#include "camera_tracker/camera_tracker.h"

namespace vslam_libs {
  namespace camera_tracker {

    CameraTracker::CameraTracker(const cv::Mat& cam_mat, cv::DescriptorMatcher* matcher)
        : cam_mat(cam_mat), matcher(matcher) {}

    int CameraTracker::recoverPose(const datastructure::FramePtr frame1,
                                   const datastructure::FramePtr frame2, cv::OutputArray R,
                                   cv::OutputArray t, cv::InputOutputArray mask) {
      // Get descriptors
      cv::Mat descriptors1 = frame1->getDescriptors();
      cv::Mat descriptors2 = frame2->getDescriptors();
      std::vector<cv::DMatch> matches;
      std::vector<char> matcher_mask;
      matcher->match(descriptors1, descriptors2, matches, matcher_mask);

      // Get keypoitns
      std::vector<cv::KeyPoint> keypoints1 = frame1->getKeypoints();
      std::vector<cv::KeyPoint> keypoints2 = frame2->getKeypoints();
      std::vector<cv::Point2f> points1, points2;
      getCorrespondences(keypoints1, keypoints2, matches, matcher_mask, points1, points2);

      // recovering the essential matrix and the pose
      cv::Mat E = cv::findEssentialMat(points2, points1, cam_mat, cv::RANSAC, 0.999, 1.0, mask);
      return cv::recoverPose(E, points2, points1, cam_mat, R, t, mask);
    }

    void getCorrespondences(const std::vector<cv::KeyPoint>& keypoints1,
                            const std::vector<cv::KeyPoint>& keypoints2,
                            const std::vector<cv::DMatch>& matches, const std::vector<char>& mask,
                            std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
      // Get the matches
      points1.clear();
      points2.clear();

      // Check if the length of matches is the same as that of mask!
      for (size_t m = 0; m < matches.size(); m++) {
        if (mask.empty() || mask[m]) {
          int i1 = matches[m].queryIdx;
          int i2 = matches[m].trainIdx;

          cv::KeyPoint kp1 = keypoints1.at(i1);
          cv::KeyPoint kp2 = keypoints2.at(i2);

          points1.emplace_back(kp1.pt);
          points2.emplace_back(kp2.pt);
        }
      }
    }
  }  // namespace camera_tracker
}  // namespace vslam_libs