// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
// To find 2D  observations from images
#include <rct_image_tools/image_observation_finder.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_static_camera.h>

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

  for (std::size_t i = 0; i < reprojections.size(); ++i)
  {
    const auto& pt = reprojections[i];
    cv::Scalar color;
    if (i == 0) {
      color = cv::Scalar(0, 255, 0);
    } else if (static_cast<int>(i) == (target.rows * target.cols - target.cols))
    {
      color = cv::Scalar(255, 0, 0);
    }
    else
      color = cv::Scalar(0,0,255);

    cv::circle(frame, pt, 3, color);
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
  ros::init(argc, argv, "camera_on_wrist_extrinsic", ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  // Load the data set path from ROS param
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  // Attempt to load the data set from the specified path
  boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  // We know it exists, so define a helpful alias
  const rct_ros_tools::ExtrinsicDataSet& data_set = *maybe_data_set;

  // Load target definition from parameter server. Target will get
  // reset if such a parameter was set.
  rct_image_tools::ModifiedCircleGridTarget target(5, 5, 0.015);
  if (!rct_ros_tools::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
  }

  // Load the camera intrinsics from the parameter server. Intr will get
  // reset if such a parameter was set
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 1411.0;
  intr.fy() = 1408.0;
  intr.cx() = 807.2;
  intr.cy() = 615.0;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
  }

  // Lets create a class that will search for the target in our raw images.
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  // Now we construct our problem:
  rct_optimizations::ExtrinsicStaticCameraMovingTargetProblem problem_def;
  // Our camera intrinsics to use
  problem_def.intr = intr;

  // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
  if (!rct_ros_tools::loadPose(pnh, "base_to_camera_guess", problem_def.base_to_camera_guess))
  {
    ROS_WARN_STREAM("Unable to load guess for base to camera from the 'base_to_camera_guess' parameter struct");
  }

//  Eigen::Matrix3d mm;
//  mm << 0,  0,  1,
//        1,  0,  0,
//        0,  1,  0;

//  problem_def.base_to_camera_guess.linear() = mm;

  ROS_WARN_STREAM("Base to camera guess:\n" << problem_def.base_to_camera_guess.matrix());

  if (!rct_ros_tools::loadPose(pnh, "wrist_to_target_guess", problem_def.wrist_to_target_guess))
  {
    ROS_WARN_STREAM("Unable to load guess for wrist to target from the 'wrist_to_target_guess' parameter struct");
  }

  ROS_WARN_STREAM("Wrist to Target guess:\n" << problem_def.wrist_to_target_guess.matrix());

  // Finally, we need to process our images into correspondence sets: for each dot in the
  // target this will be where that dot is in the target and where it was seen in the image.
  // Repeat for each image. We also tell where the wrist was when the image was taken.
  rct_ros_tools::ExtrinsicDataSet found_images;
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
    found_images.images.push_back(data_set.images[i]);
    found_images.tool_poses.push_back(data_set.tool_poses[i]);

    // So for each image we need to:
    //// 1. Record the wrist position
    problem_def.wrist_poses.push_back(data_set.tool_poses[i]);


    ROS_INFO_STREAM("WRIST POSE AT " << i << ":\n" << data_set.tool_poses[i].matrix());
    if (i != 0)
    {
      auto prev_wrist = data_set.tool_poses[i-1];
      Eigen::Vector3d delta = data_set.tool_poses[i].translation() - prev_wrist.translation();

      ROS_INFO_STREAM("DELTA SPACE: " << delta.transpose());

    }


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

  // Run optimization
  rct_optimizations::ExtrinsicStaticCameraMovingTargetResult
      opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  Eigen::Affine3d c = opt_result.base_to_camera;
  Eigen::Affine3d t = opt_result.wrist_to_target;

  std::cout << "Base to Camera:\n";
  std::cout << c.matrix() << "\n";
  std::cout << "Wrist to Target:\n";
  std::cout << t.matrix() << "\n";

  Eigen::Quaterniond qs (c.linear());
  ROS_INFO("QUAT (x, y, z, w): %f %f %f %f", qs.x(), qs.y(), qs.z(), qs.w());

  qs = Eigen::Quaterniond(t.linear());
  ROS_INFO("QUAT TARGET (x, y, z, w): %f %f %f %f", qs.x(), qs.y(), qs.z(), qs.w());


  std::cout << "--- URDF Format Base to Camera---\n";
  Eigen::Vector3d rpy = c.rotation().eulerAngles(2, 1, 0);
  std::cout << "xyz=\"" << c.translation()(0) << " " << c.translation()(1) << " " << c.translation()(2) << "\"\n";
  std::cout << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180/M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180/M_PI << " deg) " << rpy(0) << "(" << rpy(0) * 180/M_PI << " deg)\"\n";

  for (std::size_t i = 0; i < found_images.images.size(); ++i)
  {
    reproject(opt_result.wrist_to_target, opt_result.base_to_camera, found_images.tool_poses[i],
              intr, target, found_images.images[i], problem_def.image_observations[i]);
  }

  std::vector<Eigen::Affine3d> writes;
  for (std::size_t i = 0; i < found_images.images.size(); ++i)
  {
    // Solve PnP
    Eigen::Affine3d guess = Eigen::Affine3d::Identity();
    guess = guess * Eigen::Translation3d(0,0,0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    rct_optimizations::PnPProblem params;
    params.intr = intr;
    params.camera_to_target_guess = guess;
    params.correspondences = problem_def.image_observations[i];

    rct_optimizations::PnPResult r = rct_optimizations::optimize(params);
    writes.push_back(r.camera_to_target);
    std::cout << "Pnp " << i << ": " << r.converged << " " << r.final_cost_per_obs << "\n";
  }

  for (std::size_t i = 1; i < found_images.images.size(); ++i)
  {
    // Robot
    auto r1 = found_images.tool_poses[i] * problem_def.wrist_to_target_guess;
    auto r0 = found_images.tool_poses[i - 1] * problem_def.wrist_to_target_guess;

    // PnP
    auto p1 = writes[i];
    auto p0 = writes[i-1];

    // Delta
    auto dr = r0.inverse() * r1;
    auto dp = p0.inverse() * p1;

    std::cout << "DR\n" << dr.matrix() << "\n";
    std::cout << "DP\n" << dp.matrix() << "\n";
    std::cout << "TX\n" << (dr.translation() - dp.translation()).transpose() << "\n\n";

  }



  return 0;
}
