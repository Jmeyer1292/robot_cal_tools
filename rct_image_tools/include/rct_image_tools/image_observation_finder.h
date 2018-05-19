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

  rct_optimizations::Point3d point(int row, int col) const
  {
    // TODO
    rct_optimizations::Point3d p;
    p.values[0] = (rows - row  - 1)* 0.02;
    p.values[1] = (cols - col - 1) * 0.02;
    p.values[2] = 0.0;
    return p;
  }

  rct_optimizations::Point3d point(std::size_t i) const
  {
    int row = i / cols;
    int col = i % cols;

    return point(row, col);
  }
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
