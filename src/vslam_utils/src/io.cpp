#include "vslam_utils/io.hpp"

#include <filesystem>

namespace vslam_utils {
  namespace io {
    std::vector<std::string> loadFromFolder(const std::string &folder) {
      std::vector<std::string> files;

      for (const auto &f : std::filesystem::directory_iterator(folder)) {
        if (f.path().extension() == ".png" || f.path().extension() == ".jpg") {
          files.push_back(f.path());
        }
      }

      std::sort(files.begin(), files.end());

      return files;
    }
  }  // namespace io
}  // namespace vslam_utils