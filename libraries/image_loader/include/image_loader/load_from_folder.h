#ifndef __LOAD_FROM_FOLDER_H__
#define __LOAD_FROM_FOLDER_H__

#include <string>
#include <vector>
namespace vslam_libs {
  namespace image_loader {
    class LoadFromFolder {
    public:
      LoadFromFolder(const std::string& folder);

    private:
      std::vector<std::string> files;
    };

  }  // namespace image_loader
}  // namespace vslam_libs

#endif  // __LOAD_FROM_FOLDER_H__