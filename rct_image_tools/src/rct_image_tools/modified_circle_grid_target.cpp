#include "rct_image_tools/modified_circle_grid_target.h"

namespace rct_image_tools
{
ModifiedCircleGridTarget::ModifiedCircleGridTarget(const unsigned rows_, const unsigned cols_, const double spacing_)
  : rows(rows_), cols(cols_), spacing(spacing_)
{
}

std::vector<Eigen::Vector3d> ModifiedCircleGridTarget::createPoints() const
{
  std::vector<Eigen::Vector3d> points;
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

  assert(points.size() == (rows * cols));

  return points;
}

bool ModifiedCircleGridTarget::operator==(const ModifiedCircleGridTarget &other) const
{
  bool equal = true;
  equal &= (rows == other.rows);
  equal &= (cols == other.cols);
  equal &= (spacing == other.spacing);

  return equal;
}

} // namespace rct_image_tools
