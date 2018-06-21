// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_examples/data_set.h"
#include "rct_examples/parameter_loaders.h"
// To find 2D  observations from images
#include <rct_image_tools/image_observation_finder.h>
// The calibration function for 'moving camera' on robot wrist
#include <rct_optimizations/extrinsic_camera_on_wrist.h>

#include <rct_optimizations/experimental/pnp.h>

// For display of found targets
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <rct_optimizations/ceres_math_utilities.h>

static void reproject(const Eigen::Affine3d& wrist_to_camera, const Eigen::Affine3d& base_to_target,
                      const Eigen::Affine3d& base_to_wrist, const rct_optimizations::CameraIntrinsics& intr,
                      const rct_image_tools::ModifiedCircleGridTarget& target, const cv::Mat& image,
                      const rct_optimizations::CorrespondenceSet& corr)
{
  std::vector<cv::Point2d> reprojections;

  for (const auto& point_in_target : target.points)
  {
    Eigen::Vector3d in_base = (base_to_target * point_in_target);
    Eigen::Vector3d in_wrist = base_to_wrist.inverse() * in_base;
    Eigen::Vector3d in_camera =wrist_to_camera.inverse() * in_wrist;

    double uv[2];
    rct_optimizations::projectPoint(intr, in_camera.data(), uv);

    reprojections.push_back(cv::Point2d(uv[0], uv[1]));
  }

  cv::Mat frame = image.clone();

  for (const auto& pt : reprojections)
  {
    cv::circle(frame, pt, 3, cv::Scalar(0, 0, 255));
  }

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  Eigen::Affine3d cam_to_target = wrist_to_camera.inverse() * base_to_wrist.inverse() * base_to_target;
  std::cout << "CAM TO TARGET\n\n" << cam_to_target.matrix() << "\n";

  rct_optimizations::PnPProblem pb;
  pb.camera_to_target_guess = cam_to_target;
  pb.correspondences = corr;
  pb.intr = intr;


  rct_optimizations::PnPResult r = rct_optimizations::optimize(pb);
  std::cout << "PNP\n" << r.camera_to_target.matrix() << "\n";

  Eigen::Affine3d delta = r.camera_to_target * cam_to_target.inverse();
  std::cout << "DELTA S: " << delta.translation().norm() << " at " << delta.translation().transpose() << "\n";
  Eigen::AngleAxisd aa (delta.linear());
  std::cout << "DELTA A: " << (180.0 * aa.angle() / M_PI) << " and axis = " << aa.axis().transpose() << "\n";

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

  // Load target definition from parameter server. Target will get
  // reset if such a parameter was set.
  rct_image_tools::ModifiedCircleGridTarget target(10, 10, 0.0254);
  if (!rct_examples::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
  }

  // Load the camera intrinsics from the parameter server. Intr will get
  // reset if such a parameter was set
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 510.0;
  intr.fy() = 510.0;
  intr.cx() = 320.2;
  intr.cy() = 208.9;

  if (!rct_examples::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
  }

  // Attempt to load the data set via the data record yaml file:
  boost::optional<rct_examples::ExtrinsicDataSet> maybe_data_set = rct_examples::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  // We know it exists, so define a helpful alias
  const rct_examples::ExtrinsicDataSet& data_set = *maybe_data_set;

  // Lets create a class that will search for the target in our raw images.
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  // Now we create our calibration problem
  rct_optimizations::ExtrinsicCameraOnWristProblem problem_def;
  problem_def.intr = intr; // Set the camera properties

  // Provide a guess for the wrist to camera transform:
  Eigen::Vector3d wrist_to_camera_tx (0.015, 0, 0.15);
  Eigen::Matrix3d wrist_to_camera_rot;
  wrist_to_camera_rot << 0, 1, 0,
                        -1, 0, 0,
                         0, 0, 1;
  problem_def.wrist_to_camera_guess.translation() = wrist_to_camera_tx;
  problem_def.wrist_to_camera_guess.linear() = wrist_to_camera_rot;

  // Provide a guess for the base to target transform:
  Eigen::Vector3d base_to_target_tx (1, 0, 0);
  Eigen::Matrix3d base_to_target_rot;
  base_to_target_rot << 0, 1, 0,
                       -1, 0, 0,
                        0, 0, 1;
  problem_def.base_to_target_guess.translation() = base_to_target_tx;
  problem_def.base_to_target_guess.linear() = base_to_target_rot;

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

    //// Create the correspondence pairs
    rct_optimizations::CorrespondenceSet obs_set;
    assert(maybe_obs->size() == target.points.size());

    // So for each dot:
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      rct_optimizations::Correspondence2D3D pair;
      pair.in_image = maybe_obs->at(j); // The obs finder and target define their points in the same order!
      pair.in_target = target.points[j];
      obs_set.push_back(pair);
    }
    //// And finally add that to the problem
    problem_def.image_observations.push_back(obs_set);
  }

  // Now we have a defined problem, run optimization:
  rct_optimizations::ExtrinsicCameraOnWristResult opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  Eigen::Affine3d c = opt_result.wrist_to_camera;
  Eigen::Affine3d t = opt_result.base_to_target;

  std::cout << "Wrist To Camera:\n";
  std::cout << c.matrix() << "\n";
  std::cout << "Base to Target:\n";
  std::cout << t.matrix() << "\n";

  std::cout << "--- URDF Format Wrist to Camera---\n";
  Eigen::Vector3d rpy = c.rotation().eulerAngles(0, 1, 2);
  std::cout << "xyz=\"" << c.translation()(0) << " " << c.translation()(1) << " " << c.translation()(2) << "\"\n";
  std::cout << "rpy=\"" << rpy(0) << " " << rpy(1) << " " << rpy(2) << "\"\n";

  for (std::size_t i = 0; i < data_set.images.size(); ++i)
  {
    reproject(opt_result.wrist_to_camera, opt_result.base_to_target, data_set.tool_poses[i],
              intr, target, data_set.images[i], problem_def.image_observations[i]);
  }

  return 0;
}
