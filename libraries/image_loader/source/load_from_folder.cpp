#include "image_loader/load_from_folder.h"

#include <filesystem>
#include <iostream>
namespace vslam_libs {
  namespace image_loader {

    LoadFromFolder::LoadFromFolder(const std::string& folder) {
      for (const auto& f : std::filesystem::directory_iterator(folder)) {
        std::cout << "file: " << f << std::endl;
      }
    }

  }  // namespace image_loader
}  // namespace vslam_libs
