// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_examples/data_set.h"
#include "rct_examples/parameter_loaders.h"
// To find 2D  observations from images
#include <rct_image_tools/image_observation_finder.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_multi_static_camera.h>

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/imgproc.hpp>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/experimental/pnp.h>

static void reproject(const Eigen::Affine3d& wrist_to_target, const Eigen::Affine3d& base_to_camera,
                      const Eigen::Affine3d& base_to_wrist, const rct_optimizations::CameraIntrinsics& intr,
                      const rct_image_tools::ModifiedCircleGridTarget& target, const cv::Mat& image,
                      const rct_optimizations::CorrespondenceSet& corr)
{
  std::vector<cv::Point2d> reprojections;
  Eigen::Affine3d target_to_camera = wrist_to_target.inverse() * base_to_wrist.inverse() * base_to_camera;

  for (const auto& point_in_target : target.points)
  {
    Eigen::Vector3d in_camera = target_to_camera.inverse() * point_in_target;

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
  std::cout << "CAM TO TARGET\n\n" << target_to_camera.inverse().matrix() << "\n";

  rct_optimizations::PnPProblem pb;
  pb.camera_to_target_guess = target_to_camera.inverse();
  pb.correspondences = corr;
  pb.intr = intr;


  rct_optimizations::PnPResult r = rct_optimizations::optimize(pb);
  std::cout << "PNP\n" << r.camera_to_target.matrix() << "\n";

  Eigen::Affine3d delta = target_to_camera * r.camera_to_target;
  std::cout << "OTHER S: " << (r.camera_to_target.translation() - target_to_camera.translation()).norm() << "\n";
  std::cout << "DELTA S: " << delta.translation().norm() << " at " << delta.translation().transpose() << "\n";
  Eigen::AngleAxisd aa (delta.linear());
  std::cout << "DELTA A: " << (180.0 * aa.angle() / M_PI) << " and axis = " << aa.axis().transpose() << "\n";

  cv::imshow("repr", frame);
  cv::waitKey();
}
/**
 * @brief Defines a camera matrix using a camera origin, a position its looking at, and an up vector hint
 * @param origin The position of the camera focal point
 * @param eye A point that the camera is looking at
 * @param up The upvector in world-space
 */
static Eigen::Affine3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_static_camera_extrinsic");
  ros::NodeHandle pnh("~");

  sleep(10);
  int camera_count;
  if (!pnh.getParam("num_of_cameras", camera_count))
  {
    ROS_ERROR("Must set '%s' parameter", "num_of_cameras");
    return 1;
  }
  std::size_t num_of_cameras = camera_count;

  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;
  std::vector<std::string> data_path;
  std::string target_path;
  std::vector<boost::optional<rct_examples::ExtrinsicDataSet> > maybe_data_set;

  data_path.resize(num_of_cameras);
  maybe_data_set.resize(num_of_cameras);
  problem_def.intr.resize(num_of_cameras);
  problem_def.base_to_camera_guess.resize(num_of_cameras);
  problem_def.image_observations.resize(num_of_cameras);
  problem_def.wrist_poses.resize(num_of_cameras);
  for (std::size_t c = 0; c < num_of_cameras; ++c)
  {
    // Load the data set path from ROS param
    std::string param_base = "camera_" + std::to_string(c) + "/";
    std::string param_name = param_base + "data_path";
    if (!pnh.getParam(param_name, data_path[c]))
    {
      ROS_ERROR("Must set '%s' parameter", param_name.c_str());
      return 1;
    }

    // Attempt to load the data set from the specified path
    maybe_data_set[c] = rct_examples::parseFromFile(data_path[c]);
    if (!maybe_data_set[c])
    {
      ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path[c]);
      return 2;
    }

    // Load the camera intrinsics from the parameter server. Intr will get
    // reset if such a parameter was set
    param_name = param_base + "intrinsics";
    problem_def.intr[c].fx() = 1411.0;
    problem_def.intr[c].fy() = 1408.0;
    problem_def.intr[c].cx() = 807.2;
    problem_def.intr[c].cy() = 615.0;
    if (!rct_examples::loadIntrinsics(pnh, param_name, problem_def.intr[c]))
    {
      ROS_WARN("Unable to load camera intrinsics from the '%s' parameter struct", param_name.c_str());
    }

    param_name = param_base + "base_to_camera_guess";
    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    if (!rct_examples::loadPose(pnh, param_name, problem_def.base_to_camera_guess[c]))
    {
      ROS_WARN("Unable to load guess for base to camera from the '%s' parameter struct", param_name.c_str());
    }
  }

  if (!rct_examples::loadPose(pnh, "wrist_to_target_guess", problem_def.wrist_to_target_guess))
  {
    ROS_WARN_STREAM("Unable to load guess for wrist to target from the 'wrist_to_target_guess' parameter struct");
  }

  if (!pnh.getParam("target_path", target_path))
  {
    ROS_ERROR("Must set '%s' parameter", "target_path");
    return 1;
  }

  // Load target definition from parameter server. Target will get
  // reset if such a parameter was set.
  rct_image_tools::ModifiedCircleGridTarget target(5, 5, 0.015);
  if (!rct_examples::loadTarget(target_path, target))
  {
    ROS_WARN_STREAM("Unable to load target file from the 'target_path' parameter");
  }

  // Lets create a class that will search for the target in our raw images.
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  std::vector<rct_examples::ExtrinsicDataSet> found_images;
  found_images.resize(num_of_cameras);
  for (std::size_t c = 0; c < num_of_cameras; ++c)
  {
    // We know it exists, so define a helpful alias
    const rct_examples::ExtrinsicDataSet& data_set = *maybe_data_set[c];

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
      // cache found image data
      found_images[c].images.push_back(data_set.images[i]);
      found_images[c].tool_poses.push_back(data_set.tool_poses[i]);

      // So for each image we need to:
      //// 1. Record the wrist position
      problem_def.wrist_poses[c].push_back(data_set.tool_poses[i]);

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
      problem_def.image_observations[c].push_back(obs_set);
    }
  }


  // Run optimization
  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetResult
      opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";


  Eigen::Affine3d t = opt_result.wrist_to_target;

  std::cout << "Wrist to Target:\n";
  std::cout << t.matrix() << "\n";

  for (std::size_t c = 0; c < num_of_cameras; ++c)
  {
    // Load the data set path from ROS param
    std::string param_base = "camera_" + std::to_string(c);
    Eigen::Affine3d t = opt_result.base_to_camera[c];

    std::cout << "Base to Camera (" + param_base + "):\n";
    std::cout << t.matrix() << "\n";

    std::cout << "--- URDF Format Base to Camera (" + param_base + ") ---\n";
    Eigen::Vector3d rpy = t.rotation().eulerAngles(2, 1, 0);
    std::cout << "xyz=\"" << t.translation()(0) << " " << t.translation()(1) << " " << t.translation()(2) << "\"\n";
    std::cout << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180/M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180/M_PI << " deg) " << rpy(0) << "(" << rpy(0) * 180/M_PI << " deg)\"\n";
  }

  for (std::size_t c = 0; c < num_of_cameras; ++c)
  {
    for (std::size_t i = 0; i < found_images[c].images.size(); ++i)
    {
      reproject(opt_result.wrist_to_target, opt_result.base_to_camera[c], found_images[c].tool_poses[i],
                problem_def.intr[c], target, found_images[c].images[i], problem_def.image_observations[c][i]);
    }
  }

  return 0;
}
