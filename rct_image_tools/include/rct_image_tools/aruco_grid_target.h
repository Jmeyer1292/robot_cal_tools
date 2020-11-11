#pragma once

#include <rct_image_tools/target.h>

#include <opencv2/aruco.hpp>

namespace rct_image_tools
{
/**
 * @brief Structure containing relevant data for a ArUco grid target
 */
struct ArucoGridTarget : Target
{
  /**
   * @brief Constructor
   * @param rows - number of rows in the target
   * @param cols - number of columns in the target
   * @param aruco_marker_dim - The length of the side of one ArUco marker (m)
   * @param marker_gap - The size of the gap between adjacent arUco markers (m)
   * @param dictionary_id - The dictionary of ArUco markers to use
   */
  ArucoGridTarget(const int rows, const int cols, const float aruco_marker_dim, const float marker_gap,
                  const int dictionary_id = cv::aruco::DICT_6X6_250);

  /**
   * @brief Constructor
   * @param board_in - OpenCV ArUco GridBoard object defining rows, columns, marker size, and marker spacing
   */
  ArucoGridTarget(const cv::Ptr<cv::aruco::GridBoard>& board_in);

  bool operator==(const ArucoGridTarget& other) const;

  /**
   * @brief Creates a set of correspondences between the corners of each ArUco tag observed in an image (ordered clockwise from the "origin" corner)
   * and their counterparts in the target (matched by ID)
   * @param target_features - Map of ArUco tag corners observed in an image
   * @return Set of corresponding features in the image to features in the ArUco grid target
   */
  virtual rct_optimizations::Correspondence2D3D::Set createCorrespondences(const TargetFeatures& target_features) const override;

  /** @brief Representation of the ArUco grid target */
  cv::Ptr<cv::aruco::GridBoard> board;
  /** @brief Map of 3D ArUco tag corners with corresponding IDs */
  std::map<int, std::vector<Eigen::Vector3d>> points;
};

/**
 * @brief For a given ArUco GridBoard, create a map of marker corner coordinates keyed to marker IDs.
 * @param board - ArUco GridBoard to use when generating the map
 * @return Resulting map. Keys are the IDs for the ArUco markers in the board. Values are vectors containing the four board-relative coordinates
 * for the corners of the marker.
 */
static std::map<int, std::vector<Eigen::Vector3d>> mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard> &board)
{
  std::map<int, std::vector<Eigen::Vector3d>> map_ids_to_corners;
  for (std::size_t i = 0; i < board->ids.size(); i++)
  {
    std::vector<Eigen::Vector3d> obj_pts(board->objPoints[i].size());
    std::transform(
          board->objPoints[i].begin(), board->objPoints[i].end(), obj_pts.begin(),
          [](const cv::Point3f& o) -> Eigen::Vector3d { return Eigen::Vector3f(o.x, o.y, o.z).cast<double>(); });

    map_ids_to_corners.insert(std::make_pair(board->ids[i], obj_pts));
  }
  return map_ids_to_corners;
}

} // namespace rct_image_tools
