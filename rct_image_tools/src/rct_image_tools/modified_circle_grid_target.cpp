#include "rct_image_tools/modified_circle_grid_target.h"

rct_image_tools::ModifiedCircleGridTarget::ModifiedCircleGridTarget(int rows, int cols, double spacing)
    : rows(rows), cols(cols), x_spacing(spacing), y_spacing(spacing)
{
  makePoints(rows, cols, spacing, points);
}

bool rct_image_tools::ModifiedCircleGridTarget::makePoints(std::size_t rows, std::size_t cols, double spacing,
                                                           std::vector<Eigen::Vector3d>& points)
{
  points.reserve(rows * cols);

  for (std::size_t i = 1; i < (rows + 1); i++)
  {
    double y = (rows - i) * spacing;
    for (std::size_t j = 0; j < cols; j++)
    {
      double x = j * spacing;
      Eigen::Vector3d point(x, y, 0.0);
      points.push_back(point);
    }
  }

  // TODO(gChiou): May need a better check...
  if (points.size() == (rows * cols))
  {
    return true;
  }
  else
  {
    return false;
  }
}
