#include <rct_image_tools/charuco_finder.h>

rct_image_tools::CharucoGridBoardObservationFinder::CharucoGridBoardObservationFinder(const cv::Ptr<cv::aruco::CharucoBoard>& board)
	: board_(board)
{

}

std::map<int, std::vector<Eigen::Vector2f>>
rct_image_tools::CharucoGridBoardObservationFinder::findObservations(const cv::Mat &image) const
{
	try {
    std::map<int, std::vector<Eigen::Vector2f>> map_ids_to_obs_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->doCornerRefinement = false;
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f> > marker_corners, rejected_candidates;
		cv::aruco::detectMarkers(image, board_->dictionary, marker_corners, marker_ids, parameters);
		
		std::vector<cv::Point2f> charuco_corners;
		std::vector<int> charuco_ids;
		cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board_, charuco_corners, charuco_ids);

    for(int i = 0; i < charuco_ids.size(); i++)
    {
      std::vector<Eigen::Vector2f> obs_pts(1);
      obs_pts[0] << charuco_corners[i].x, charuco_corners[i].y;
      map_ids_to_obs_corners.insert(std::make_pair(charuco_ids[i], obs_pts));
		}
		return map_ids_to_obs_corners;	
	}
  catch(...)
	{
    throw "Error in finding and returning ChAruco corners";
	}
}

cv::Mat
rct_image_tools::CharucoGridBoardObservationFinder::drawObservations(const cv::Mat& image,
                                                                   const std::map<int, std::vector<Eigen::Vector2d>>& observations) const
{
  std::vector<int> charuco_ids(observations.size());
  std::vector<std::vector<cv::Point2f>> charuco_corners(observations.size());
  for(auto it = observations.begin(); it != observations.end(); ++it)
	{
    charuco_ids.push_back(it->first);
		std::vector<cv::Point2f> cv_obs(it->second.size());
		std::transform(it->second.begin(), it->second.end(), cv_obs.begin(), [](const Eigen::Vector2d& o) {return cv::Point2d(o.x(), o.y()); });
    charuco_corners.push_back(cv_obs);
  }
  cv::aruco::drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
  return image;
}

/*std::vector<rct_optimizations::Correspondence2D3D>
rct_image_tools::createCorrespondences(const cv::Ptr<cv::aruco::CharucoBoard>& board)
{
  std::vector<rct_optimizations::Correspondence2D3D> map_ids_to_corners;
  for (int i = 0; i < board->ids.size(); i++)
  {
    std::vector<rct_optimizations::Correspondence2D3D> obj_pts(board->objPoints[i].size());
    std::transform(board->objPoints[i].begin(), board->objPoints[i].end(), obj_pts.begin(), [](const cv::Point3f& o) {return Eigen::Vector3d(o.x, o.y, o.z); });
    map_ids_to_corners.insert(std::make_pair(board->ids[i], obj_pts));
  }
  return map_ids_to_corners;
}*/

