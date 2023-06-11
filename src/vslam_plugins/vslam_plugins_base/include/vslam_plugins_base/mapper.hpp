#ifndef VSLAM_PLUGINS_BASE__MAPPER_HPP_
#define VSLAM_PLUGINS_BASE__MAPPER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_mapper {
  namespace base {
    class Mapper : public virtual vslam_plugin::base::Plugin {
    public:
      /// Mapper initializer
      virtual void initialize() = 0;

      /// Create new map points based on the point correspondences and the pose constraints
      /**
       * \param[in] matched_points Point correspndences
       * \param[in] T_1_w camera pose of the first frane
       * \param[in] T_2_1 camera pose from the first to second frame
       * \param[in] K camera matrix
       */
      virtual vslam_datastructure::MapPoints map(vslam_datastructure::MatchedPoints& matched_points,
                                                 const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K)
          = 0;

      /// Destructor
      virtual ~Mapper() {}

    protected:
      /// Constructor
      Mapper() {}
    };
  }  // namespace base
}  // namespace vslam_mapper

#endif  // VSLAM_PLUGINS_BASE__MAPPER_HPP_