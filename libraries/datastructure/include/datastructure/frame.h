#ifndef __DATASTRUCTURE_H__
#define __DATASTRUCTURE_H__

#include <opencv2/core.hpp>

namespace vslam_libs {
  namespace datastructure {

    class Frame {
    public:
      explicit Frame(const cv::Mat& image);

    private:
      cv::Mat image;  //<! the image of the frame
    };

  }  // namespace datastructure
}  // namespace vslam_libs

#endif  // __DATASTRUCTURE_H__