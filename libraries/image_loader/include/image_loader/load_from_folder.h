#ifndef __LOAD_FROM_FOLDER_H__
#define __LOAD_FROM_FOLDER_H__

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace vslam_libs {
  namespace image_loader {
    class LoadFromFolder {
    public:
      LoadFromFolder(const std::string& folder, std::string ext = ".png");

      cv::Mat getNextFrame();

    private:
      unsigned int i;                  //!< pointer to the current frame in the files list
      std::vector<std::string> files;  //!< Files in a folder
    };

  }  // namespace image_loader
}  // namespace vslam_libs

#endif  // __LOAD_FROM_FOLDER_H__