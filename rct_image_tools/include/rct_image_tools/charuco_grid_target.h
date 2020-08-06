#pragma once

#include <rct_optimizations/types.h>

#include <Eigen/Dense>
#include <map>
#include <opencv2/aruco/charuco.hpp>

namespace rct_image_tools
{
/**
 * @brief Structure containing relevant data for a ChArUco grid target
 */
struct CharucoGridTarget
{
  /**
   * @brief Constructor
   * @param cols - number of columns in the target
   * @param rows - number of rows in the target
   * @param chessboard_dim - The length of the side of one chessboard square (m)
   * @param aruco_marker_dim - The length of the side of one ArUco marker (m)
   * @param dictionary_id - The dictionary of ArUco markers to use
   */
  CharucoGridTarget(const int cols, const int rows, const double chessboard_dim, const double aruco_marker_dim, const int dictionary_id = cv::aruco::DICT_6X6_250)
  {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);

    // Create charuco board object
    board = cv::aruco::CharucoBoard::create(cols, rows, chessboard_dim, aruco_marker_dim, dictionary);

    // Create a map of chessboard intersection IDs and their location on the board
    for (std::size_t i = 0; i < board->chessboardCorners.size(); i++)
    {
      const auto &corner = board->chessboardCorners.at(i);
      Eigen::Vector3f pt(corner.x, corner.y, corner.z);
      points.emplace(i, pt.cast<double>());
    }
  }

  /**
   * @brief Creates a set of correspondences between chessboard intersections observed in an image and their counterparts
   * in the target (matched by ID)
   * @param features - Map of observed chessboard intersections and their IDs
   * @return Set of corresponding features in the image to features in the ChArUco target
   */
  std::vector<rct_optimizations::Correspondence2D3D>
  createCorrespondences(const std::map<int, Eigen::Vector2d>& features) const
  {
    std::vector<rct_optimizations::Correspondence2D3D> correspondences;
    correspondences.reserve(features.size());

    for(auto it = features.begin(); it != features.end(); it++)
    {
      rct_optimizations::Correspondence2D3D corr;
      corr.in_target = points.at(it->first);
      corr.in_image = features.at(it->first);
      correspondences.push_back(corr);
    }
    return correspondences;
  }

  /** @brief Representation of the ChArUco board target */
  cv::Ptr<cv::aruco::CharucoBoard> board;
  /** @brief Map of 3D chessboard corners with corresponding IDs */
  std::map<int, Eigen::Vector3d> points;
};

} // namespace rct_image_tools
