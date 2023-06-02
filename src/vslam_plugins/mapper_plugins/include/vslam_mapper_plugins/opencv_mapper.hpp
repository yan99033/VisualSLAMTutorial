#ifndef VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
#define VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_

#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/mapper.hpp"

namespace vslam_mapper_plugins {

  class OpenCVMapper : public virtual vslam_mapper::base::Mapper {
  public:
    void initialize() override {}

    virtual vslam_datastructure::MapPoints map(vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& T_1_w,
                                               const cv::Mat& T_2_1, const cv::Mat& K) override;

  private:
    static constexpr const double proj_err_thresh_{8.0};
  };
}  // namespace vslam_mapper_plugins

#endif  // VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
