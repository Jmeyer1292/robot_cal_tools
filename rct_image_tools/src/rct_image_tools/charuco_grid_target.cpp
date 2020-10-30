#include <rct_image_tools/charuco_grid_target.h>

namespace rct_image_tools
{
CharucoGridTarget::CharucoGridTarget(const int rows, const int cols, const double chessboard_dim,
                                     const double aruco_marker_dim, const int dictionary_id)
{
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);

  // Create charuco board object
  board = cv::aruco::CharucoBoard::create(cols, rows, chessboard_dim, aruco_marker_dim, dictionary);

  // Create a map of chessboard intersection IDs and their location on the board
  for (std::size_t i = 0; i < board->chessboardCorners.size(); i++)
  {
    const auto& corner = board->chessboardCorners.at(i);
    Eigen::Vector3f pt(corner.x, corner.y, corner.z);
    points.emplace(i, pt.cast<double>());
  }
}

bool CharucoGridTarget::operator==(const CharucoGridTarget& other) const
{
  auto board_size = board->getChessboardSize();
  auto other_board_size = other.board->getChessboardSize();
  bool equal = true;
  equal &= other.points == points;
  equal &= other.board->getMarkerLength() == board->getMarkerLength();
  equal &= other.board->getSquareLength() == board->getSquareLength();
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
