/**
  * ChAruco gridboard detector, following the same pattern as ModifiedCircleGridObservationFinder.
  * Author: John Berkebile
  */
#pragma once

#include <rct_image_tools/charuco_grid_target.h>

namespace rct_image_tools
{

/**
 * @brief This class finds 2D observations from images of a specified ChArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 */
class CharucoGridBoardObservationFinder
{
public:
  CharucoGridBoardObservationFinder(const CharucoGridTarget& target);

  /**
   * @brief Detects chessboard intersection coordinates in the provided image.
   * @param image - Input image, ideally containing a ChArUco grid target.
   * @return Map matching marker ID numbers to the 2D position of the chessboard intersections
   */
  std::map<int, Eigen::Vector2d> findObservations(const cv::Mat& image) const;

  /**
   * @brief A debugging utility that will draw an observation set onto a copy of a cv::Mat for
   * display purposes
   * @param image - Input image, ideally containing a ChArUco grid target
   * @param observations - Observations of the chessboard intersections (obtained by calling @ref findObservations)
   * @return An image with the chessboard intersections and IDs overlaid on the input image
   */
  cv::Mat drawObservations(const cv::Mat& image, const std::map<int, Eigen::Vector2d>& observations) const;

  const CharucoGridTarget& target() const { return target_; }

private:
  CharucoGridTarget target_;
};

} // namespace rct_image_tools

