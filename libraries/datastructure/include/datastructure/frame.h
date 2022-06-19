#ifndef __DATASTRUCTURE_H__
#define __DATASTRUCTURE_H__

#include <memory>
#include <opencv2/core.hpp>
namespace vslam_libs {
  namespace datastructure {
    class Frame {
    public:
      explicit Frame(const cv::Mat& image);
      Frame(const cv::Mat& image, const cv::Mat& cam_mat);

    private:
      cv::Mat image;    //<! the image of the frame
      cv::Mat cam_mat;  //<! 3x3 camera matrix containing the camera intrinsics
    };

    using FramePtr = std::shared_ptr<Frame>;

  }  // namespace datastructure
}  // namespace vslam_libs

#endif  // __DATASTRUCTURE_H__