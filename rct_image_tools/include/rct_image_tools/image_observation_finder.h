#ifndef IMAGE_OBSERVATION_FINDER_H
#define IMAGE_OBSERVATION_FINDER_H

#include <opencv2/core.hpp>
#include <rct_optimizations/types.h>
#include <boost/optional.hpp>
#include <Eigen/Dense>

namespace rct_image_tools
{

struct TargetDefinition
{
  int rows;
  int cols;
  std::vector<Eigen::Vector3d> points;

  bool makePoints(std::size_t rows, std::size_t cols, double spacing, std::vector<Eigen::Vector3d> &points)
  {
    points.reserve(rows*cols);

    for (std::size_t i = 1; i < (rows+1); i++)
    {
      double y = (rows-i)*spacing;
      for (std::size_t j = 0; j < cols; j++)
      {
        double x = j*spacing;
        Eigen::Vector3d point(x, y, 0.0);
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

  boost::optional<std::vector<Eigen::Vector2d>> findObservations(const cv::Mat& image, cv::Mat& out) const;

private:
  TargetDefinition target_;
};

}

#endif // IMAGE_OBSERVATION_FINDER_H
