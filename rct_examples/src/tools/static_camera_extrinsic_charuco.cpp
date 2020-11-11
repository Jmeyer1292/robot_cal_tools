// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/target_loaders.h"
#include "rct_ros_tools/print_utils.h"
#include <rct_image_tools/image_utils.h>

// This header brings in a tool for finding the target in a given image
#include <rct_image_tools/charuco_finder.h>

// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_hand_eye.h>

// Calibration Validation
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/pnp.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

static Eigen::Isometry3d reproject(const Eigen::Isometry3d& camera_to_target,
                                   const Correspondence2D3D::Set& correspondence_set,
                                   const CameraIntrinsics& intr,
                                   const rct_image_tools::CharucoGridTarget& target, const cv::Mat& image)
{
  std::vector<Eigen::Vector3d> target_pts;
  target_pts.reserve(target.points.size());
  for(auto it = target.points.begin(); it != target.points.end(); it++)
  {
    target_pts.push_back(target.points.at(it->first));
  }

  std::vector<cv::Point2d> reprojections = rct_image_tools::getReprojections(camera_to_target, intr, target_pts);

  cv::Mat frame = image.clone();
  rct_image_tools::drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = correspondence_set;
  pb.intr = intr;
  PnPResult r = optimize(pb);

  cv::imshow("repr", frame);
  cv::waitKey();

  return r.camera_to_target;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_camera_extrinsic_charuco");
  ros::NodeHandle pnh("~");

  // Load the data set path from ROS param
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  // Attempt to load the data set from the specified path
  boost::optional<ExtrinsicDataSet> maybe_data_set = parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  // We know it exists, so define a helpful alias
  const ExtrinsicDataSet& data_set = *maybe_data_set;

  try
  {
    // Load the target definition
    auto target = TargetLoader<CharucoGridTarget>::load(pnh, "target_definition");

    // Load the camera intrinsics
    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Lets create a class that will search for the target in our raw images.
    rct_image_tools::CharucoGridBoardTargetFinder target_finder(target);

    // Now we construct our problem:
    ExtrinsicHandEyeProblem2D3D problem_def;

    // Our camera intrinsics to use
    problem_def.intr = intr;

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    problem_def.camera_mount_to_camera_guess = loadPose(pnh, "base_to_camera_guess");
    problem_def.target_mount_to_target_guess = loadPose(pnh, "wrist_to_target_guess");

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    ExtrinsicDataSet found_images;
    problem_def.observations.reserve(data_set.images.size());

    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the circle grid in this image:
      auto observations = target_finder.findTargetFeatures(data_set.images[i]);
      if (observations.empty())
      {
        ROS_WARN_STREAM("Unable to find the ChAruco grid in image: " << i);
        cv::imshow("points", data_set.images[i]);
        cv::waitKey();
        continue;
      }
      else
      {
        // Show the points we detected
        cv::imshow("points", target_finder.drawTargetFeatures(data_set.images[i], observations));
        cv::waitKey();
      }

      // cache found image data
      found_images.images.push_back(data_set.images[i]);
      found_images.tool_poses.push_back(data_set.tool_poses[i]);

      Observation2D3D obs;

      // So for each image we need to:
      //// 1. Record the wrist position
      obs.to_target_mount = data_set.tool_poses[i];
      obs.to_camera_mount = Eigen::Isometry3d::Identity();

      //// And finally add that to the problem
      obs.correspondence_set = target.createCorrespondences(observations);

      problem_def.observations.push_back(obs);
    }

    // Run the optimization
    ExtrinsicHandEyeResult opt_result = optimize(problem_def);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
    printTransform(c, "Base", "Camera", "BASE TO CAMERA");
    printNewLine();

    Eigen::Isometry3d t = opt_result.target_mount_to_target;
    printTransform(t, "Wrist", "Target", "WRIST_TO_TARGET");
    printNewLine();

    // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
    // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic parameters
    // We will then see how much this transform differs from the same transform calculated using the results of the extrinsic calibration

    // Create accumulators to more easily calculate the mean and standard deviation of the position and orientation differences
    namespace ba = boost::accumulators;
    ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_diff_acc;
    ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_diff_acc;

    // Iterate over all of the images in which an observation of the target was made
    for (std::size_t i = 0; i < found_images.images.size(); ++i)
    {
      // Get the observation
      const Observation2D3D& obs = problem_def.observations.at(i);

      // Calculate the optimized transform from the camera to the target for the ith observation
      Eigen::Isometry3d camera_to_target =
          opt_result.camera_mount_to_camera.inverse() * (obs.to_target_mount * opt_result.target_mount_to_target);

      // Get the same transformation from a PnP optimization with the known camera intrinsic parameters
      Eigen::Isometry3d camera_to_target_pnp =
          reproject(camera_to_target, obs.correspondence_set, intr, target, found_images.images[i]);

      // Calculate the difference between the two transforms
      Eigen::Isometry3d diff = camera_to_target.inverse() * camera_to_target_pnp;

      // Accumulate the differences
      pos_diff_acc(diff.translation().norm());
      ori_diff_acc(Eigen::Quaterniond(camera_to_target.linear())
                     .angularDistance(Eigen::Quaterniond(camera_to_target_pnp.linear())));
    }

    ROS_INFO_STREAM("Difference in camera to target transform between extrinsic calibration and PnP optimization");
    ROS_INFO_STREAM("Position:\n\tMean (m): "
                    << ba::mean(pos_diff_acc) << "\n\tStd. Dev. (m): " << std::sqrt(ba::variance(pos_diff_acc)));
    ROS_INFO_STREAM("Orientation:\n\tMean (rad): "
                    << ba::mean(ori_diff_acc) << "\n\tStd. Dev. (rad): " << std::sqrt(ba::variance(ori_diff_acc)));

  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}




