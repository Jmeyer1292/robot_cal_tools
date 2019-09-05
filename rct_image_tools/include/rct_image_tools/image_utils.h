#ifndef RCT_IMAGE_UTILS_H
#define RCT_IMAGE_UTILS_H
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_image_tools/image_observation_finder.h>
#include <vector>

namespace rct_image_tools
{

/**
 * @brief Get the uv coordinates of a point reprojected into the image frame
 * @param camera_to_target The transformation from the camera frame to the target frame
 * @param intr The intrinsic values of the camera
 * @param target_points A vector of target points
 * @return A vector of uv values in the image frame
 */
inline
std::vector<cv::Point2d> getReprojections(const Eigen::Affine3d &camera_to_target,
                                          const rct_optimizations::CameraIntrinsics &intr,
                                          const std::vector<Eigen::Vector3d> &target_points)
{
  std::vector<cv::Point2d> reprojections;
  for (const auto& point_in_target : target_points)
  {
    Eigen::Vector3d in_camera = camera_to_target * point_in_target;

    double uv[2];
    rct_optimizations::projectPoint(intr, in_camera.data(), uv);

    reprojections.push_back(cv::Point2d(uv[0], uv[1]));
  }
  return reprojections;
}

/**
 * @brief Draw a set of reprojections on an image
 * @param reprojections A vector of reprojections to be drawn on the image
 * @param size The size of circle to drawn
 * @param color The color of the circle drawn
 * @param image The image to draw the reprojections on
 */
inline
void drawReprojections(const std::vector<cv::Point2d> &reprojections,
                       double size,
                       cv::Scalar color,
                       cv::Mat &image)
{
  for (const auto& pt : reprojections)
  {
    cv::circle(image, pt, size, color);
  }
}

/**
 * @brief Get the correspondence set given observations in the image and 3d target points
 * @param observations Observation in the image frame
 * @param target_points The 3d target points
 * @return The correspondence set
 */
inline
rct_optimizations::CorrespondenceSet getCorrespondenceSet(const std::vector<Eigen::Vector2d> &observations, const std::vector<Eigen::Vector3d> &target_points)
{
  rct_optimizations::CorrespondenceSet obs_set;

  //// Create the correspondence pairs
  assert(observations.size() == target_points.size());

  // So for each dot:
  for (std::size_t j = 0; j < observations.size(); ++j)
  {
    rct_optimizations::Correspondence2D3D pair;
    pair.in_image = observations.at(j); // The obs finder and target define their points in the same order!
    pair.in_target = target_points.at(j);

    obs_set.push_back(pair);
  }

  return obs_set;
}

/**
 * @brief Get a correspondence set given the observation finder and an image
 * @param obs_finder The observation finder
 * @param image The image to search for observation
 * @return A correspondence set
 */
inline
rct_optimizations::CorrespondenceSet getCorrespondenceSet(const rct_image_tools::ModifiedCircleGridObservationFinder &obs_finder, const cv::Mat& image)
{
  const rct_image_tools::ModifiedCircleGridTarget& target = obs_finder.target();
  auto maybe_obs = obs_finder.findObservations(image);

  rct_optimizations::CorrespondenceSet obs_set;
  if (maybe_obs)
  {
    obs_set = getCorrespondenceSet(*maybe_obs, target.points);
  }

  return obs_set;
}

}
#endif // RCT_IMAGE_UTILS_H
