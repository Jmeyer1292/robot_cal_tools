#ifndef CHARUCO_GRID_TARGET_H
#define CHARUCO_GRID_TARGET_H

#include <Eigen/Dense>
#include <map>
#include <opencv2/aruco/charuco.hpp>
#include <rct_optimizations/types.h>

namespace rct_image_tools
{

struct CharucoGridTarget
{
  CharucoGridTarget(const int cols, const int rows, const double squareLength, const double markerLength)
  {
    const int dictionaryId = 10;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Define Target - create charuco board object
    charucoboard = cv::aruco::CharucoBoard::create(cols, rows, squareLength, markerLength, dictionary);
    int ids = ((cols - 1) * (rows - 1));          // The (rows - 2) accounts for the sides that do not have intersecting corners
    auto corners = charucoboard->chessboardCorners;
    for (std::size_t i = 0; i < ids; i++)
    {
      const auto &c = corners[i];
      Eigen::Vector3f pt(c.x, c.y, c.z);
      points.emplace(i, pt.cast<double>());
    }
  }



  /**
   * @brief createCorrespondences Utility function to map marker ID values to the spatial coordinates of the corners
   * defined by the definition of the gridboard. This is used to generate a set of "object points" which can be used
   * in conjunction with the set of "image points" generated by findObservations to match 2D image coordinates with
   * corresponding 3D object coordinates.
   * @param board ChArUco gridboard to use.
   * @return Map matching marker ID numbers to the 2D position of the chessboard corners
   * from a ChArUco board using the detected Aruco markers coordinates.
   */
  std::vector<rct_optimizations::Correspondence2D3D>
  createCorrespondences(const std::map<int, Eigen::Vector2d>& features) const
  {
    std::vector<rct_optimizations::Correspondence2D3D> map_ids_to_corners;
    map_ids_to_corners.reserve(features.size());

    for(auto it = features.begin(); it != features.end(); it++)
    {
      rct_optimizations::Correspondence2D3D corr;
      corr.in_target = points.at(it->first);
      corr.in_image = features.at(it->first);
      map_ids_to_corners.push_back(corr);
    }
    return map_ids_to_corners;
  }

  std::map<int, Eigen::Vector3d> points;
  cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
};
}

#endif // CHARUCO_GRID_TARGET_H
