/**
  * ChAruco gridboard detector, following the same pattern as ModifiedCircleGridTargetFinder.
  * Author: John Berkebile
  */
#pragma once

#include <rct_image_tools/target_finder.h>
#include <rct_image_tools/charuco_grid_target.h>

namespace rct_image_tools
{

/**
 * @brief This class finds 2D features from images of a specified ChArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 */
class CharucoGridBoardTargetFinder : public TargetFinder
{
public:
  CharucoGridBoardTargetFinder(const CharucoGridTarget& target);

  /**
   * @brief Detects chessboard intersection coordinates in the provided image.
   * @param image - Input image, ideally containing a ChArUco grid target.
   * @return Map matching marker ID numbers to the 2D position of the chessboard intersections
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw targer features set onto an input image for display purposes
   * @param image - Input image, ideally containing a ChArUco grid target
   * @param target_features - Chessboard intersections (obtained by calling @ref findTargetFeatures)
   * @return An image with the chessboard intersections and IDs overlaid on the input image
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;

  virtual const Target& target() const override
  {
    return target_;
  }

protected:
  const CharucoGridTarget target_;
};

} // namespace rct_image_tools

