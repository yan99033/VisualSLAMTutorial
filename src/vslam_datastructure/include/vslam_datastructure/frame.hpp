#ifndef VSLAM_DATASTRUCTURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_HPP_

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace vslam_datastructure {
  class Frame {
  public:
    explicit Frame(const cv::Mat& image);

    // Getters
    void getPose(Sophus::SE3d& Tcw);
    cv::Mat getImage() const;

    // Setter
    void setPose(const Sophus::SE3d& Tcw);

  private:
    cv::Mat image;  //<! the image of the frame

    long unsigned int id;

    static long unsigned int frame_count;

    // Camera pose
    Sophus::SE3d Tcw;

    std::mutex mutex;
  };

  long unsigned int Frame::frame_count = 0;

  using FramePtr = std::shared_ptr<Frame>;

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
