#include <rct_image_tools/aruco_grid_target.h>

namespace
{
/**
 * @brief For a given ArUco GridBoard, create a map of marker corner coordinates keyed to marker IDs.
 * @param board - ArUco GridBoard to use when generating the map
 * @return Resulting map. Keys are the IDs for the ArUco markers in the board. Values are vectors containing the four board-relative coordinates
 * for the corners of the marker.
 */
std::map<int, std::vector<Eigen::Vector3d>> mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard> &board)
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
}

namespace rct_image_tools
{
ArucoGridTarget::ArucoGridTarget(const int rows, const int cols, const float aruco_marker_dim, const float marker_gap,
                                 const int dictionary_id)
  : ArucoGridTarget(cv::aruco::GridBoard::create(cols, rows, aruco_marker_dim, marker_gap, cv::aruco::getPredefinedDictionary(dictionary_id)))
{
}

ArucoGridTarget::ArucoGridTarget(const cv::Ptr<cv::aruco::GridBoard>& board_in)
  : board(board_in)
  , points(mapArucoIdsToObjPts(board))
{
}

bool ArucoGridTarget::operator==(const ArucoGridTarget& other) const
{
  auto board_size = board->getGridSize();
  auto other_board_size = other.board->getGridSize();
  bool equal = true;
  equal &= other_board_size.width == board_size.width;
  equal &= other_board_size.height == board_size.height;
  equal &= (std::abs(other.board->getMarkerLength() - board->getMarkerLength()) < std::numeric_limits<float>::epsilon());
  equal &= (std::abs(other.board->getMarkerSeparation() - board->getMarkerSeparation()) < std::numeric_limits<float>::epsilon());
  equal &= other.points == points;
  return equal;
}

rct_optimizations::Correspondence2D3D::Set
ArucoGridTarget::createCorrespondences(const TargetFeatures& target_features) const
{
  rct_optimizations::Correspondence2D3D::Set correspondences;
  correspondences.reserve(target_features.size());

  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    for (std::size_t i = 0; i < it->second.size(); ++i)
    {
      rct_optimizations::Correspondence2D3D corr;
      corr.in_target = points.at(it->first).at(i);
      corr.in_image = target_features.at(it->first).at(i);
      correspondences.push_back(corr);
    }
  }
  return correspondences;
}

} // namespace rct_image_tools
