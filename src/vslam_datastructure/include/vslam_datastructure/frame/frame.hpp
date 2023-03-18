#ifndef VSLAM_DATASTRUCTURE__FEATURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FEATURE__FRAME_HPP_

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace vslam_datastructure {

  namespace frame {
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
      cv::Mat image;    //<! the image of the frame
      cv::Mat cam_mat;  //<! 3x3 camera matrix containing the camera intrinsics

      // Camera pose
      Sophus::SE3d Tcw;

      std::mutex mutex;
    };

    using FramePtr = std::shared_ptr<Frame>;

  }  // namespace frame

}  // namespace vslam_datastructure

#endif
