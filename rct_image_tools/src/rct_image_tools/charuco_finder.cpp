#include <rct_image_tools/charuco_finder.h>
#include <rct_image_tools/charuco_grid_target.h>

#include <opencv2/aruco/charuco.hpp>

rct_image_tools::CharucoGridBoardObservationFinder::CharucoGridBoardObservationFinder(const CharucoGridTarget& target)
  : target_(target)
{
}

std::map<int, Eigen::Vector2d>
rct_image_tools::CharucoGridBoardObservationFinder::findObservations(const cv::Mat& image) const
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
  std::map<int, Eigen::Vector2d> features;
  for (int i = 0; i < detected_corners; i++)
  {
    const cv::Point2f& corner = charuco_corners.at(i);
    Eigen::Vector2d obs_pts(corner.x, corner.y);
    features.emplace(charuco_ids[i], obs_pts);
  }

  return features;
}

cv::Mat rct_image_tools::CharucoGridBoardObservationFinder::drawObservations(
    const cv::Mat& image, const std::map<int, Eigen::Vector2d>& observations) const
{
  std::vector<int> charuco_ids;
  charuco_ids.reserve(observations.size());

  std::vector<cv::Point2f> charuco_corners;
  charuco_corners.reserve(observations.size());

  for(auto it = observations.begin(); it != observations.end(); ++it)
  {
    // Add the ID
    charuco_ids.push_back(it->first);

    // Add the image coordinates
    cv::Point2f cv_obs(it->second.x(), it->second.y());
    charuco_corners.push_back(cv_obs);
  }

  // Draw the detected corners
  cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

  return image;
}
