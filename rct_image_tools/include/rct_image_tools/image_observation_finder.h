#ifndef IMAGE_OBSERVATION_FINDER_H
#define IMAGE_OBSERVATION_FINDER_H

#include "rct_image_tools/modified_circle_grid_target.h"
#include <boost/optional.hpp>
#include <opencv2/core.hpp>
#include <rct_optimizations/types.h>

namespace rct_image_tools
{

class ImageObservationFinder
{
public:
  ImageObservationFinder(const ModifiedCircleGridTarget& target);

  boost::optional<std::vector<Eigen::Vector2d>> findObservations(const cv::Mat& image) const;

  cv::Mat drawObservations(const cv::Mat& image, const std::vector<Eigen::Vector2d>& observations) const;

private:
  ModifiedCircleGridTarget target_;
};
}

#endif // IMAGE_OBSERVATION_FINDER_H
