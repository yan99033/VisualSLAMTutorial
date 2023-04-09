#ifndef VSLAM_PLUGINS_BASE__BACKEND_HPP_
#define VSLAM_PLUGINS_BASE__BACKEND_HPP_

namespace vslam_backend_base {
  class Backend {
  public:
    virtual void initialize() = 0;
    virtual ~Backend() {}

  protected:
    Backend() {}
  };
}  // namespace vslam_backend_base

#endif  // VSLAM_PLUGINS_BASE__BACKEND_HPP_