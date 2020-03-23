/**
  * ArUco gridboard detector, following the same pattern as ModifiedCircleGridObservationFinder.
  * Author: Joseph Schornak
  */

#include "rct_image_tools/aruco_finder.h"

rct_image_tools::ArucoGridBoardObservationFinder::ArucoGridBoardObservationFinder(const cv::Ptr<cv::aruco::GridBoard>& board)
    : board_(board)
{

}

boost::optional<std::map<int, std::vector<Eigen::Vector2d>>>
rct_image_tools::ArucoGridBoardObservationFinder::findObservations(const cv::Mat &image) const
{
  std::map<int, std::vector<Eigen::Vector2d>> map_ids_to_obs_corners;

  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> marker_ids;
  cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);

  cv::aruco::detectMarkers(image, board_->dictionary, marker_corners, marker_ids, parameters, rejected_candidates);
  cv::aruco::refineDetectedMarkers(image, board_, marker_corners, marker_ids, rejected_candidates);

  for(int i = 0; i < marker_ids.size(); i++)
  {
    std::vector<cv::Point2f> corner_pts = marker_corners[i];

    std::vector<Eigen::Vector2d> obs_pts(4);
    for (int j = 0; j < corner_pts.size(); j++)
    {
      obs_pts[j] << corner_pts[j].x, corner_pts[j].y;
    }
    map_ids_to_obs_corners.insert(std::make_pair(marker_ids[i], obs_pts));
  }
  return map_ids_to_obs_corners;
}

cv::Mat
rct_image_tools::ArucoGridBoardObservationFinder::drawObservations(const cv::Mat& image,
                                                                   const std::map<int, std::vector<Eigen::Vector2d>>& observations) const
{
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  for(std::map<int, std::vector<Eigen::Vector2d>>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    marker_ids.push_back(it->first);
    std::vector<cv::Point2f> cv_obs(it->second.size());
    std::transform(it->second.begin(), it->second.end(), cv_obs.begin(), [](const Eigen::Vector2d& o) {return cv::Point2d(o.x(), o.y()); });
    marker_corners.push_back(cv_obs);
  }
  cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
  return image;
}

std::map<int, std::vector<Eigen::Vector3d>>
rct_image_tools::mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard> &board)
{
  std::map<int, std::vector<Eigen::Vector3d>> map_ids_to_corners;
  for (int i = 0; i < board->ids.size(); i++)
  {
    std::vector<Eigen::Vector3d> obj_pts(board->objPoints[i].size());
    std::transform(board->objPoints[i].begin(), board->objPoints[i].end(), obj_pts.begin(), [](const cv::Point3f& o) {return Eigen::Vector3d(o.x, o.y, o.z); });
    map_ids_to_corners.insert(std::make_pair(board->ids[i], obj_pts));
  }
  return map_ids_to_corners;
}
