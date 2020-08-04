#include <rct_image_tools/charuco_finder.h>
#include <rct_image_tools/charuco_grid_target.h>

rct_image_tools::CharucoGridBoardObservationFinder::CharucoGridBoardObservationFinder(const CharucoGridTarget target)
  : target_(target)
{

}

std::map<int, Eigen::Vector2d>
rct_image_tools::CharucoGridBoardObservationFinder::findObservations(const cv::Mat &image) const
{
    std::map<int, Eigen::Vector2d> map_ids_to_obs_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners, rejected_candidates;
    cv::aruco::detectMarkers(image, target_.charucoboard->dictionary, marker_corners, marker_ids, parameters);
		
    std::vector<cv::Point2f> charuco_corners;
		std::vector<int> charuco_ids;
    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, target_.charucoboard, charuco_corners, charuco_ids);

    for(int i = 0; i < charuco_ids.size(); i++)
    {
      Eigen::Vector2d obs_pts;
      obs_pts << charuco_corners[i].x, charuco_corners[i].y;
      map_ids_to_obs_corners.emplace(charuco_ids[i], obs_pts);
		}
		return map_ids_to_obs_corners;	
}

cv::Mat
rct_image_tools::CharucoGridBoardObservationFinder::drawObservations(const cv::Mat& image,
                                                                   const std::map<int, Eigen::Vector2d>& observations) const
{
  std::vector<int> charuco_ids(observations.size());
  std::vector<cv::Point2f> charuco_corners(observations.size());
  for(auto it = observations.begin(); it != observations.end(); ++it)
	{
    charuco_ids.push_back(it->first);
    cv::Point2f cv_obs(it->second.x(), it->second.y());
    charuco_corners.push_back(cv_obs);
  }
  cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
  return image;
}
