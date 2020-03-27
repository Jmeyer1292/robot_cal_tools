// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/print_utils.h"
// To find 2D  observations from images
#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
// The calibration function for 'moving camera' on robot wrist
#include <rct_optimizations/extrinsic_camera_on_wrist.h>
#include <rct_optimizations/experimental/pnp.h>

// For display of found targets
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <rct_optimizations/ceres_math_utilities.h>

static void reproject(const Eigen::Isometry3d& wrist_to_camera, const Eigen::Isometry3d& base_to_target,
                      const Eigen::Isometry3d& base_to_wrist, const rct_optimizations::CameraIntrinsics& intr,
                      const rct_image_tools::ModifiedCircleGridTarget& target, const cv::Mat& image,
                      const rct_optimizations::CorrespondenceSet& corr)
{
  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  Eigen::Isometry3d camera_to_target = (base_to_wrist * wrist_to_camera).inverse() * base_to_target;
  std::vector<cv::Point2d> reprojections = rct_image_tools::getReprojections(camera_to_target, intr, target.points);

  cv::Mat frame = image.clone();
  rct_image_tools::drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);

  rct_optimizations::PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = corr;
  pb.intr = intr;
  rct_optimizations::PnPResult r = rct_optimizations::optimize(pb);

  rct_ros_tools::printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTransform(r.camera_to_target, "Camera", "Target", "PNP");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTransformDiff(camera_to_target, r.camera_to_target, "Camera", "Target", "PNP DIFF");
  rct_ros_tools::printNewLine();

  cv::imshow("repr", frame);
  cv::waitKey();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  // Load the data set path from ROS param
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  rct_image_tools::ModifiedCircleGridTarget target;
  if (!rct_ros_tools::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
    return 1;
  }

  rct_optimizations::CameraIntrinsics intr;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
    return 1;
  }

  // Attempt to load the data set via the data record yaml file:
  boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  // We know it exists, so define a helpful alias
  const rct_ros_tools::ExtrinsicDataSet& data_set = *maybe_data_set;

  // Lets create a class that will search for the target in our raw images.
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  // Now we create our calibration problem
  rct_optimizations::ExtrinsicCameraOnWristProblem problem_def;
  problem_def.intr = intr; // Set the camera properties

  // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
  if (!rct_ros_tools::loadPose(pnh, "base_to_target_guess", problem_def.base_to_target_guess))
  {
    ROS_WARN_STREAM("Unable to load guess for base to camera from the 'base_to_target_guess' parameter struct");
    return 1;
  }

  if (!rct_ros_tools::loadPose(pnh, "wrist_to_camera_guess", problem_def.wrist_to_camera_guess))
  {
    ROS_WARN_STREAM("Unable to load guess for wrist to target from the 'wrist_to_camera_guess' parameter struct");
    return 1;
  }

  // Finally, we need to process our images into correspondence sets: for each dot in the
  // target this will be where that dot is in the target and where it was seen in the image.
  // Repeat for each image. We also tell where the wrist was when the image was taken.
  for (std::size_t i = 0; i < data_set.images.size(); ++i)
  {
    // Try to find the circle grid in this image:
    auto maybe_obs = obs_finder.findObservations(data_set.images[i]);
    if (!maybe_obs)
    {
      ROS_WARN_STREAM("Unable to find the circle grid in image: " << i);
      cv::imshow("points", data_set.images[i]);
      cv::waitKey();
      continue;
    }
    else
    {
      // Show the points we detected
      cv::imshow("points", obs_finder.drawObservations(data_set.images[i], *maybe_obs));
      cv::waitKey();
    }

    // So for each image we need to:
    //// 1. Record the wrist position
    problem_def.wrist_poses.push_back(data_set.tool_poses[i]);

    //// And finally add that to the problem
    problem_def.image_observations.push_back(rct_image_tools::getCorrespondenceSet(*maybe_obs, target.points));
  }

  // Now we have a defined problem, run optimization:
  rct_optimizations::ExtrinsicCameraOnWristResult opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  rct_ros_tools::printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  Eigen::Isometry3d c = opt_result.wrist_to_camera;
  rct_ros_tools::printTransform(c, "Wrist", "Camera", "WRIST TO CAMERA");
  rct_ros_tools::printNewLine();

  Eigen::Isometry3d t = opt_result.base_to_target;
  rct_ros_tools::printTransform(t, "Base", "Target", "BASE TO TARGET");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTitle("REPROJECTION ERROR");
  for (std::size_t i = 0; i < data_set.images.size(); ++i)
  {
    rct_ros_tools::printTitle("REPROJECT IMAGE " + std::to_string(i));
    reproject(opt_result.wrist_to_camera, opt_result.base_to_target, data_set.tool_poses[i],
              intr, target, data_set.images[i], problem_def.image_observations[i]);
  }

  return 0;
}
