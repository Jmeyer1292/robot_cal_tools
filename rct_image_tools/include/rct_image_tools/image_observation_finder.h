#ifndef RCT_IMAGE_OBSERVATION_FINDER_H
#define RCT_IMAGE_OBSERVATION_FINDER_H

#include "rct_image_tools/modified_circle_grid_target.h"
#include <vector>
#include <boost/optional.hpp>
#include <opencv2/core.hpp>
#include <rct_optimizations/types.h>

namespace rct_image_tools
{

/**
 * @brief This class finds 2D observations from images of a known ModifiedCircleGridTarget.
 * All points must be seen or it will fail. Observations are returned in the same order
 * as points are defined in the target.
 */
class ModifiedCircleGridObservationFinder
{
public:
  ModifiedCircleGridObservationFinder(const ModifiedCircleGridTarget& target);

  boost::optional<std::vector<Eigen::Vector2d>> findObservations(const cv::Mat& image) const;

  /**
   * @brief A debugging utility that will draw an observation set onto a copy of a cv::Mat for
   * display purposes. Usually you want to call findObservations() above then this with the result.
   */
  cv::Mat drawObservations(const cv::Mat& image, const std::vector<Eigen::Vector2d>& observations) const;

  const ModifiedCircleGridTarget& target() const { return target_; }

private:
  ModifiedCircleGridTarget target_;
};
}

#endif // RCT_IMAGE_OBSERVATION_FINDER_H
