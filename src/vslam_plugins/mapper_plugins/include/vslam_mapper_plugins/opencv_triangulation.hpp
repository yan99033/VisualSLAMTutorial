#ifndef VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
#define VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_

#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/mapper.hpp"

namespace vslam_mapper_plugins {

  class OpenCvTriangulation : public vslam_mapper_base::Mapper {
  public:
    void initialize(const cv::Mat& K) override;

    virtual void map(vslam_datastructure::MatchedPoints& matched_points) override;

  protected:
    cv::Mat K;
  };
}  // namespace vslam_mapper_plugins

#endif  // VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
