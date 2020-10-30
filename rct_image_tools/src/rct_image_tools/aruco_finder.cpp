/**
  * ArUco gridboard detector, following the same pattern as ModifiedCircleGridTargetFinder.
  * Author: Joseph Schornak
  */

#include "rct_image_tools/aruco_finder.h"

namespace rct_image_tools
{
ArucoGridBoardTargetFinder::ArucoGridBoardTargetFinder(const ArucoGridTarget& target)
  : TargetFinder(target)
{
}

TargetFeatures ArucoGridBoardTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  TargetFeatures map_ids_to_obs_corners;

  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> marker_ids;
  cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);

  cv::aruco::detectMarkers(image, target_.board->dictionary, marker_corners, marker_ids, parameters, rejected_candidates);
  cv::aruco::refineDetectedMarkers(image, target_.board, marker_corners, marker_ids, rejected_candidates);

  for (unsigned i = 0; i < marker_ids.size(); i++)
  {
    std::vector<cv::Point2f> corner_pts = marker_corners[i];

    std::vector<Eigen::Vector2d> obs_pts(4);
    for (unsigned j = 0; j < corner_pts.size(); j++)
    {
      obs_pts[j] << corner_pts[j].x, corner_pts[j].y;
    }
    map_ids_to_obs_corners.insert(std::make_pair(marker_ids[i], obs_pts));
  }
  return map_ids_to_obs_corners;
}

cv::Mat ArucoGridBoardTargetFinder::drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const
{
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  for(auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    marker_ids.push_back(it->first);
    std::vector<cv::Point2f> cv_obs(it->second.size());
    std::transform(it->second.begin(), it->second.end(), cv_obs.begin(), [](const Eigen::Vector2d& o) {return cv::Point2d(o.x(), o.y()); });
    marker_corners.push_back(cv_obs);
  }
  cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
  return image;
}

std::map<unsigned, std::vector<Eigen::Vector3d>> mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard> &board)
{
  std::map<unsigned, std::vector<Eigen::Vector3d>> map_ids_to_corners;
  for (unsigned i = 0; i < board->ids.size(); i++)
  {
    std::vector<Eigen::Vector3d> obj_pts(board->objPoints[i].size());
    std::transform(board->objPoints[i].begin(), board->objPoints[i].end(), obj_pts.begin(), [](const cv::Point3f& o) {return Eigen::Vector3d(o.x, o.y, o.z); });
    map_ids_to_corners.insert(std::make_pair(board->ids[i], obj_pts));
  }
  return map_ids_to_corners;
}

} // namespace rct_image_tools
