#ifndef RCT_MODIFIED_CIRCLE_GRID_TARGET_H
#define RCT_MODIFIED_CIRCLE_GRID_TARGET_H

#include <Eigen/Dense>

namespace rct_image_tools
{

struct ModifiedCircleGridTarget
{
  ModifiedCircleGridTarget() = default;
  ModifiedCircleGridTarget(int rows, int cols, double spacing);

  int rows = 0;
  int cols = 0;
  double x_spacing = 0.0;
  double y_spacing = 0.0;
  std::vector<Eigen::Vector3d> points;

private:
  bool makePoints(std::size_t rows, std::size_t cols, double spacing, std::vector<Eigen::Vector3d>& points);
};
}

#endif // RCT_MODIFIED_CIRCLE_GRID_TARGET_H
