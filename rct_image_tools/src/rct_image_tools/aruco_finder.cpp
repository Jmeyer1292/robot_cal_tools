/**
  * ArUco gridboard detector, following the same pattern as ModifiedCircleGridTargetFinder.
  * Author: Joseph Schornak
  */

#include "rct_image_tools/aruco_finder.h"

namespace rct_image_tools
{
ArucoGridBoardTargetFinder::ArucoGridBoardTargetFinder(const ArucoGridTarget& target)
  : TargetFinder()
  , target_(target)
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
    VectorEigenVector<2> obs_pts(4);

    for (unsigned j = 0; j < corner_pts.size(); j++)
    {
      obs_pts[j] = Eigen::Vector2d(corner_pts[j].x, corner_pts[j].y).cast<double>();
    }
    map_ids_to_obs_corners.emplace(marker_ids[i], obs_pts);
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
} // namespace rct_image_tools
