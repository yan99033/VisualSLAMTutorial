#ifndef VSLAM_PLUGINS_BASE__BASE_HPP_
#define VSLAM_PLUGINS_BASE__BASE_HPP_

#include <string>

namespace vslam_plugin {
  namespace base {
    class Plugin {
    public:
      /// Get the plugin name
      virtual inline std::string getPluginName() = 0;
    };
  }  // namespace base

}  // namespace vslam_plugin

#endif  // VSLAM_PLUGINS_BASE__BASE_HPP_