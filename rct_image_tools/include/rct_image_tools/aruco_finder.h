#pragma once

#include <rct_image_tools/target_finder.h>
#include <rct_image_tools/aruco_grid_target.h>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

namespace rct_image_tools
{

/**
 * @brief This class finds 2D target features from images of a specified ArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 * Target features are returned as a map where the marker ID is the key and the image coordinates of the
 * marker corners are the mapped value.
 */
class ArucoGridBoardTargetFinder : public TargetFinder<ArucoGridTarget>
{
public:
  ArucoGridBoardTargetFinder(const ArucoGridTarget& target);

  /**
   * @brief Detect marker corner coordinates in the provided image.
   * @param image - Input image, ideally containing an ArUco gridboard.
   * @return Map matching marker ID numbers to a vector of marker corner coordinates. The vector will contain
   * four corners defined in the same order as in the output of the function cv::aruco::DetectMarkers()
   * (e.g. clockwise from the "origin" corner).
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw a set of target features onto an image for
   * display purposes. Usually you want to call findTargetFeatures() above then this with the result.
   * @param image - The image of the target
   * @param target_features - The target features identified in the input image
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;
};

} // namespace rct_image_tools
