#ifndef VSLAM_PLUGINS_BASE__BASE_HPP_
#define VSLAM_PLUGINS_BASE__BASE_HPP_

#include <string>

namespace vslam_plugin {
  namespace base {
    class Plugin {
    public:
      virtual inline std::string get_plugin_name() = 0;
    };
  }  // namespace base

}  // namespace vslam_plugin

#endif  // VSLAM_PLUGINS_BASE__BASE_HPP_