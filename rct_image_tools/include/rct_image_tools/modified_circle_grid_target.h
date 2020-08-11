#ifndef RCT_MODIFIED_CIRCLE_GRID_TARGET_H
#define RCT_MODIFIED_CIRCLE_GRID_TARGET_H

#include <Eigen/Dense>
#include <vector>

namespace rct_image_tools
{
/**
 * @brief Structure containing the necessary data to represent a modified circle grid target
 */
struct ModifiedCircleGridTarget
{
  ModifiedCircleGridTarget() = default;
  ModifiedCircleGridTarget(const unsigned rows, const unsigned cols, const double spacing);

  bool operator==(const ModifiedCircleGridTarget& other) const;

  unsigned rows;
  unsigned cols;
  double spacing;
  std::vector<Eigen::Vector3d> createPoints() const;
};

} // namespace rct_image_tools

#endif // RCT_MODIFIED_CIRCLE_GRID_TARGET_H
