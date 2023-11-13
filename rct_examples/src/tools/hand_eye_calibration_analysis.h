#pragma once

#include <rct_optimizations/pnp.h>
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_image_tools/image_utils.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <opencv2/highgui.hpp>
#include <ros/console.h>

using namespace rct_optimizations;
using namespace rct_image_tools;

/**
 * @brief Reprojects target points into the image using the calibrated transform and performs a PnP optimization for
 * comparison to the calibrated camera to target transform
 * @param camera_to_target - calibrated transformation from the camera frame to the target frame
 * @param correspondence_set
 * @param intr
 * @param image
 * @param window_name
 * @return
 */
Eigen::Isometry3d reproject(const Eigen::Isometry3d& camera_to_target,
                            const Correspondence2D3D::Set& correspondence_set,
                            const CameraIntrinsics& intr,
                            const cv::Mat& image,
                            const std::string& window_name)
{
  std::vector<Eigen::Vector3d> target_points;
  target_points.reserve(correspondence_set.size());
  std::transform(correspondence_set.begin(),
                 correspondence_set.end(),
                 std::back_inserter(target_points),
                 [](const rct_optimizations::Correspondence2D3D& corr) { return corr.in_target; });
  std::vector<cv::Point2d> reprojections = getReprojections(camera_to_target, intr, target_points);

  cv::Mat frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = correspondence_set;
  pb.intr = intr;
  PnPResult r = optimize(pb);

  cv::imshow(window_name, frame);
  cv::waitKey();

  return r.camera_to_target;
}

/**
 * @brief Analyzes the results of the hand eye calibration by measuring the difference between the calibrated camera to
 * target transform and a PnP optimization estimation of the same transform
 * @param problem
 * @param opt_result
 * @param images
 * @param window_name
 */
void analyzeResults(const ExtrinsicHandEyeProblem2D3D& problem,
                    const ExtrinsicHandEyeResult& opt_result,
                    const std::vector<cv::Mat>& images,
                    const std::string& window_name)
{
  // Create accumulators to more easily calculate the mean and standard deviation of the position and orientation
  // differences
  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_diff_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_diff_acc;

  // Iterate over all of the images in which an observation of the target was made
  for (std::size_t i = 0; i < images.size(); ++i)
  {
    // Get the observation
    const Observation2D3D& obs = problem.observations.at(i);

    // Calculate the optimized transform from the camera to the target for the ith observation
    Eigen::Isometry3d camera_to_target = opt_result.camera_mount_to_camera.inverse() * obs.to_camera_mount.inverse() *
                                         obs.to_target_mount * opt_result.target_mount_to_target;

    // Get the same transformation from a PnP optimization with the known camera intrinsic parameters
    Eigen::Isometry3d camera_to_target_pnp =
        reproject(camera_to_target, obs.correspondence_set, problem.intr, images[i], window_name);

    // Calculate the difference between the two transforms
    Eigen::Isometry3d diff = camera_to_target.inverse() * camera_to_target_pnp;

    // Accumulate the differences
    pos_diff_acc(diff.translation().norm());
    ori_diff_acc(Eigen::Quaterniond(camera_to_target.linear())
                     .angularDistance(Eigen::Quaterniond(camera_to_target_pnp.linear())));
  }

  ROS_INFO_STREAM("Difference in camera to target transform between extrinsic calibration and PnP optimization");
  ROS_INFO_STREAM("Position:\n\tMean (m): " << ba::mean(pos_diff_acc)
                                            << "\n\tStd. Dev. (m): " << std::sqrt(ba::variance(pos_diff_acc)));
  ROS_INFO_STREAM("Orientation:\n\tMean (deg): " << ba::mean(ori_diff_acc) * 180.0 / M_PI << "\n\tStd. Dev. (deg): "
                                                 << std::sqrt(ba::variance(ori_diff_acc)) * 180.0 / M_PI);
}
