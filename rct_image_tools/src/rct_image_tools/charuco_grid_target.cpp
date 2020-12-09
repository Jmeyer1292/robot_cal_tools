#include <rct_image_tools/charuco_grid_target.h>

namespace
{
/**
 * @brief For a given ChArUco board, create a map of chessboard intersection coordinates keyed to marker IDs.
 * @param board - ChArUco board to use when generating the map
 * @return Resulting map. Keys are the indices of chessboard intersections in the board. Values are board-relative coordinates for each intersection.
 */
std::map<unsigned, Eigen::Vector3d> mapCharucoIdsToObjPts(const cv::Ptr<cv::aruco::CharucoBoard> &board)
{
  std::map<unsigned, Eigen::Vector3d> points;
  for (std::size_t i = 0; i < board->chessboardCorners.size(); i++)
  {
    const auto& corner = board->chessboardCorners.at(i);
    Eigen::Vector3f pt(corner.x, corner.y, corner.z);
    points.emplace(i, pt.cast<double>());
  }
  return points;
}
}

namespace rct_image_tools
{
CharucoGridTarget::CharucoGridTarget(const int rows, const int cols, const float chessboard_dim,
                                     const float aruco_marker_dim, const int dictionary_id)
  : CharucoGridTarget::CharucoGridTarget(cv::aruco::CharucoBoard::create(cols, rows, chessboard_dim, aruco_marker_dim, cv::aruco::getPredefinedDictionary(dictionary_id)))
{
}

CharucoGridTarget::CharucoGridTarget(const cv::Ptr<cv::aruco::CharucoBoard>& board_in)
  : board(board_in)
  , points(mapCharucoIdsToObjPts(board))
{
}

bool CharucoGridTarget::operator==(const CharucoGridTarget& other) const
{
  auto board_size = board->getChessboardSize();
  auto other_board_size = other.board->getChessboardSize();
  bool equal = true;
  equal &= other.points == points;
  equal &= (std::abs(other.board->getMarkerLength() - board->getMarkerLength()) < std::numeric_limits<float>::epsilon());
  equal &= (std::abs(other.board->getSquareLength() - board->getSquareLength()) < std::numeric_limits<float>::epsilon());
  equal &= other_board_size.width == board_size.width;
  equal &= other_board_size.height == board_size.height;
  return equal;
}

std::vector<rct_optimizations::Correspondence2D3D>
CharucoGridTarget::createCorrespondences(const TargetFeatures& target_features) const
{
  std::vector<rct_optimizations::Correspondence2D3D> correspondences;
  correspondences.reserve(target_features.size());

  for(auto it = target_features.begin(); it != target_features.end(); it++)
  {
    rct_optimizations::Correspondence2D3D corr;
    corr.in_target = points.at(it->first);
    // Get the first (and only) feature from the current iterator
    corr.in_image = target_features.at(it->first).at(0);
    correspondences.push_back(corr);
  }
  return correspondences;
}

} // namespace rct_image_tools
