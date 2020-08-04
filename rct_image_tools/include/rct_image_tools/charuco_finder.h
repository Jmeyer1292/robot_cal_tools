/**
  * ChAruco gridboard detector, following the same pattern as ModifiedCircleGridObservationFinder.
  * Author: John Berkebile
  */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <rct_optimizations/types.h>
#include <map>
#include <Eigen/Core>
#include <iostream>
#include <rct_image_tools/charuco_grid_target.h>

namespace rct_image_tools
{

/**
 * @brief This class finds 2D observations from images of a specified ChArUco gridboard target.
 * The main advantage of this kind of target is that partial views still provide usable correspondences.
 * Observations are returned as a map where the marker ID is the key and the image coordinates of the
 * marker corners are the mapped value.
 */
class CharucoGridBoardObservationFinder
{
public:
  CharucoGridBoardObservationFinder(const CharucoGridTarget);

  /**
   * @brief findObservations Detect marker corner coordinates in the provided image.
   * @param image Input image, ideally containing an ChArUco gridboard.
   * @return Map matching marker ID numbers to the 2D position of the chessboard corners
   * from a ChArUco board using the detected Aruco markers coordinates. 
   */
  std::map<int, Eigen::Vector2d> findObservations(const cv::Mat& image) const;

  /**
   * @brief A debugging utility that will draw an observation set onto a copy of a cv::Mat for
   * display purposes. Usually you want to call findObservations() above then this with the result.
   */
  cv::Mat drawObservations(const cv::Mat& image, const std::map<int, Eigen::Vector2d>& observations) const;

  const CharucoGridTarget& target() const { return target_; }

private:
  CharucoGridTarget target_;
};

}

