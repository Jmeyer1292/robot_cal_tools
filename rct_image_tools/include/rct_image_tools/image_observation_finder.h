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
  std::vector<rct_optimizations::Point3d> points;

  bool makePoints(std::size_t rows, std::size_t cols, double spacing, std::vector<rct_optimizations::Point3d> &points)
  {
    points.reserve(rows*cols);

    for (std::size_t i = 1; i < (rows+1); i++)
    {
      double y = (rows-i)*spacing;
      for (std::size_t j = 0; j < cols; j++)
      {
        double x = j*spacing;
        rct_optimizations::Point3d point;
        point.values[0] = x;
        point.values[1] = y;
        point.values[2] = 0.0;
        points.push_back(point);
      }
    }

    // TODO(gChiou): May need a better check...
    if (points.size() == (rows*cols)) {return true;}
    else {return false;}
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
