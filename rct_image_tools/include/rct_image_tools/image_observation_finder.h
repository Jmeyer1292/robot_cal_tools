#ifndef IMAGE_OBSERVATION_FINDER_H
#define IMAGE_OBSERVATION_FINDER_H

#include <opencv2/core.hpp>
#include <rct_optimizations/types.h>
#include <boost/optional.hpp>

namespace rct_image_tools
{

struct TargetDefinition
{
  int rows;
  int cols;
};

class ImageObservationFinder
{
public:
  ImageObservationFinder(const TargetDefinition& definition);

  boost::optional<std::vector<rct_optimizations::Observation2d>> findObservations(const cv::Mat& image, cv::Mat& out) const;

private:
  TargetDefinition target_;
};

}

#endif // IMAGE_OBSERVATION_FINDER_H
