#pragma once

#include <rct_image_tools/target_features.h>
#include <rct_optimizations/types.h>

namespace rct_image_tools
{
/**
 * @brief Base class for calibration target definitions
 */
struct Target
{
  Target() = default;
  virtual ~Target() = default;

  /**
   * @brief Creates a set of correspondences between an input set of target features (identified in a 2D image) and the same features from the known geometry of the target
   * @param target_features - map of target features identified in a 2D image
   * @return
   */
  virtual rct_optimizations::Correspondence2D3D::Set
    createCorrespondences(const TargetFeatures& target_features) const = 0;
};

} // namespace rct_image_tools
