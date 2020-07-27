/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <rct_image_tools/charuco.hpp>
#include <rct_image_tools/charuco_finder.h>

rct_image_tools::CharucoGridBoardObservationFinder::CharucoGridBoardObservationFinder(const cv::Ptr<cv::aruco::CharucoBoard>& board)
	: board_(board)
{

}

boost::optional<std::map<int, std::vector<Eigen::Vector2d>>>
rct_image_tools::CharucoGridBoardObservationFinder::findObservations(const cv::Mat &image) const
{
	std::map<int, std::vector<Eigen::Vector2d>> map_ids_to_obs_corners;
	cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);
	std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners, rejected_candidates;
    cv::aruco::detectMarkers(image, board_->dictionary, marker_corners, marker_ids, parameters);
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
rct_image_tools::CharucoGridBoardObservationFinder::drawObservations(const cv::Mat& image,
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
	std::vector<cv::Point2f> charuco_corners;
	std::vector<int> charuco_ids;
	cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board_, charuco_corners, charuco_ids);
	cv::aruco::drawDetectedCornersCharuco(imageCopy, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
  	return image;
}

std::map<int, std::vector<Eigen::Vector3d>>
rct_image_tools::mapCharucoIdsToObjPts(const cv::Ptr<cv::aruco::CharucoBoard> &board)
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

