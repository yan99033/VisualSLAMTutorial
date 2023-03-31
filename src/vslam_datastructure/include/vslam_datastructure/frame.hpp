#ifndef VSLAM_DATASTRUCTURE__FRAME_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_HPP_

#include <memory>
#include <opencv2/core.hpp>

namespace vslam_datastructure {
  class Frame {
  public:
    explicit Frame(const cv::Mat& image);

    // Getters
    cv::Mat getPose() const;
    cv::Mat getImage() const;

    // Setter
    void setPose(const cv::Mat& Tcw);

  private:
    cv::Mat image;  //<! the image of the frame

    long unsigned int id;

    static long unsigned int frame_count;

    // Camera pose
    cv::Mat Tcw;

    std::mutex mutex;
  };

  using FramePtr = std::shared_ptr<Frame>;

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_HPP_
