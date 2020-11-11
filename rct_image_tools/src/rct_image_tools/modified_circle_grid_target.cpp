#include "rct_image_tools/modified_circle_grid_target.h"

namespace rct_image_tools
{
ModifiedCircleGridTarget::ModifiedCircleGridTarget(const unsigned rows_, const unsigned cols_, const double spacing_)
  : rows(rows_), cols(cols_), spacing(spacing_)
{
}

bool ModifiedCircleGridTarget::operator==(const ModifiedCircleGridTarget &other) const
{
  bool equal = true;
  equal &= (rows == other.rows);
  equal &= (cols == other.cols);
  equal &= (std::abs(spacing - other.spacing) < std::numeric_limits<double>::epsilon());

  return equal;
}

rct_optimizations::Correspondence2D3D::Set ModifiedCircleGridTarget::createCorrespondences(const TargetFeatures &target_features) const
{
  std::vector<Eigen::Vector3d> target_points = createPoints();
  assert(target_features.size() == target_points.size());

  rct_optimizations::Correspondence2D3D::Set correspondences;
  correspondences.reserve(target_features.size());

  for (std::size_t i = 0; i < target_points.size(); ++i)
  {
    rct_optimizations::Correspondence2D3D corr;
    corr.in_target = target_points.at(i);
    // Get the first (and only) element of the features at the current index; increment the index
    corr.in_image = target_features.at(static_cast<const unsigned>(i)).at(0);
    correspondences.push_back(corr);
  }

  return correspondences;
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

} // namespace rct_image_tools
