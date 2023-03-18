#ifndef VSLAM_DATASTRUCTURE__FEATURE__BASE_HPP_
#define VSLAM_DATASTRUCTURE__FEATURE__BASE_HPP_

namespace vslam_datastructure {

  namespace feature {

    struct Feature {
      enum class Type { undefined = 0, orb = 1 };
    };

  }  // namespace feature

}  // namespace vslam_datastructure

#endif
