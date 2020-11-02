#pragma once

#include <rct_image_tools/target.h>

namespace rct_image_tools
{
/**
 * @brief Structure containing the necessary data to represent a modified circle grid target
 */
struct ModifiedCircleGridTarget : Target
{
  /**
   * @brief Constructor
   * @param rows - Number of rows in the target
   * @param cols - Number of columns in the target
   * @param spacing - The spacing between adjacent circle centers (m)
   */
  ModifiedCircleGridTarget(const unsigned rows, const unsigned cols, const double spacing);

  bool operator==(const ModifiedCircleGridTarget& other) const;

  virtual rct_optimizations::Correspondence2D3D::Set
    createCorrespondences(const TargetFeatures& target_features) const override;

  std::vector<Eigen::Vector3d> createPoints() const;

  unsigned rows;
  unsigned cols;
  double spacing;
};

} // namespace rct_image_tools
