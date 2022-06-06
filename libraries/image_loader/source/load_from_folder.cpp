#include "image_loader/load_from_folder.h"

#include <fmt/core.h>

#include <algorithm>
#include <filesystem>
#include <gsl/assert>
namespace vslam_libs {
  namespace image_loader {

    LoadFromFolder::LoadFromFolder(const std::string& folder, std::string ext) {
      for (const auto& f : std::filesystem::directory_iterator(folder)) {
        if (f.path().extension() == ext) {
          files.push_back(f.path());
        }
      }

      std::sort(files.begin(), files.end());

      Ensures(!files.empty());

      fmt::print("Loaded {} files\n", files.size());
    }
  }  // namespace image_loader
}  // namespace vslam_libs
