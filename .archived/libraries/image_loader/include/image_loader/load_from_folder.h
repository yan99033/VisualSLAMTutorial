#ifndef __LOAD_FROM_FOLDER_H__
#define __LOAD_FROM_FOLDER_H__

#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace vslam_libs {
  namespace image_loader {
    class LoadFromFolder {
    public:
      explicit LoadFromFolder(const std::string& folder, std::string ext = ".png");

      cv::Mat getNextFrame();

    private:
      unsigned int i;                  //!< pointer to the current frame in the files list
      std::vector<std::string> files;  //!< Files in a folder
    };

    class ImageLoaderError : public std::runtime_error {
    public:
      ImageLoaderError(const std::string msg) : runtime_error(msg) {}
    };

  }  // namespace image_loader
}  // namespace vslam_libs

#endif  // __LOAD_FROM_FOLDER_H__