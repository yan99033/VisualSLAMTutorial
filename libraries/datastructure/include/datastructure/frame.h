#ifndef __DATASTRUCTURE_H__
#define __DATASTRUCTURE_H__

#include <memory>
#include <opencv2/core.hpp>
namespace vslam_libs {
  namespace datastructure {

    class Frame {
    public:
      explicit Frame(const cv::Mat& image);

    private:
      cv::Mat image;  //<! the image of the frame
    };

    using FramePtr = std::shared_ptr<Frame>;

  }  // namespace datastructure
}  // namespace vslam_libs

#endif  // __DATASTRUCTURE_H__