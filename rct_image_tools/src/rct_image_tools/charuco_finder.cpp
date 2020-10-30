#include <rct_image_tools/charuco_finder.h>
#include <rct_image_tools/charuco_grid_target.h>

#include <opencv2/aruco/charuco.hpp>

namespace rct_image_tools
{
CharucoGridBoardTargetFinder::CharucoGridBoardTargetFinder(const CharucoGridTarget& target)
  : TargetFinder(target)
{
}

TargetFeatures CharucoGridBoardTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  // Create a generic set of parameters
  // TODO: expose the setting of these parameters
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

  // Detect the ArUco markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(image, target_.board->dictionary, marker_corners, marker_ids, parameters);

  // Detect the chessboard intersections given the observed ArUco markers
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  int detected_corners = cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, target_.board,
                                                              charuco_corners, charuco_ids);

  // Create the map of observed features
  TargetFeatures target_features;
  for (int i = 0; i < detected_corners; i++)
  {
    const cv::Point2f& corner = charuco_corners.at(i);
    std::vector<Eigen::Vector2d> v_obs;
    v_obs.push_back(Eigen::Vector2d (corner.x, corner.y));
    target_features.emplace(static_cast<unsigned>(charuco_ids[i]), v_obs);
  }

  return target_features;
}

cv::Mat CharucoGridBoardTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                         const TargetFeatures& target_features) const
{
  std::vector<int> charuco_ids;
  charuco_ids.reserve(target_features.size());

  std::vector<cv::Point2f> charuco_corners;
  charuco_corners.reserve(target_features.size());

  for(auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    // Add the ID
    charuco_ids.push_back(it->first);

    // Add the image coordinates
    const Eigen::Vector2d& pt = it->second.at(0);
    charuco_corners.push_back(cv::Point2f(pt.x(), pt.y()));
  }

  // Draw the detected corners
  cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

  return image;
}

} // namespace rct_image_tools

