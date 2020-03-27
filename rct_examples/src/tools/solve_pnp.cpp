/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/experimental/pnp.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/print_utils.h>

static Eigen::Isometry3d solveCVPnP(const rct_optimizations::CameraIntrinsics& intr,
                                  const rct_image_tools::ModifiedCircleGridTarget& target,
                                  const std::vector<Eigen::Vector2d>& obs)
{
  cv::Mat cam_matrix (3, 3, cv::DataType<double>::type);
  cv::setIdentity(cam_matrix);

  cam_matrix.at<double>(0, 0) = intr.fx();
  cam_matrix.at<double>(1, 1) = intr.fy();
  cam_matrix.at<double>(0, 2) = intr.cx();
  cam_matrix.at<double>(1, 2) = intr.cy();

  rct_ros_tools::printCameraIntrinsics(intr.values, "Camera Intrinsics");
  rct_ros_tools::printNewLine();

  std::vector<cv::Point2d> image_points;
  for (const auto o : obs)
    image_points.push_back(cv::Point2d(o(0), o(1)));

  std::vector<cv::Point3d> target_points;
  for (const auto p : target.points)
    target_points.push_back( cv::Point3d(p(0), p(1), p(2)) );

  cv::Mat rvec (3, 1, cv::DataType<double>::type);
  cv::Mat tvec (3, 1, cv::DataType<double>::type);
  cv::solvePnP(target_points, image_points, cam_matrix, cv::noArray(), rvec, tvec);

  Eigen::Vector3d rr (Eigen::Vector3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)));
  Eigen::Isometry3d result(Eigen::AngleAxisd(rr.norm(), rr.normalized()));
  result.translation() = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

  rct_ros_tools::printTransform(result, "Camera", "Target", "OpenCV solvePNP");
  rct_ros_tools::printNewLine();

  return result;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "solve_pnp_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  // Load the image
  std::string image_path;
  if (!pnh.getParam("image_path", image_path))
  {
    ROS_ERROR_STREAM("Must set the 'image_path' private parameter");
    return 1;
  }
  cv::Mat mat = cv::imread(image_path);

  // Load target definition from parameter server
  rct_image_tools::ModifiedCircleGridTarget target;
  if (!rct_ros_tools::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
    return 1;
  }

  // Load the camera intrinsics from the parameter server
  rct_optimizations::CameraIntrinsics intr;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
    return 1;
  }

  rct_image_tools::ModifiedCircleGridObservationFinder finder (target);
  auto maybe_obs = finder.findObservations(mat);
  if (!maybe_obs)
  {
    std::cerr << "Unable to detect the target!\n";
    return 1;
  }
  cv::Mat show = finder.drawObservations(mat, *maybe_obs);
  cv::imshow("win", show);
  cv::waitKey();

  // Solve with OpenCV
  solveCVPnP(intr, target, *maybe_obs);

  // Solve with some native RCT function (for learning)
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess = guess * Eigen::Translation3d(0,0,0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  rct_optimizations::PnPProblem params;
  params.intr = intr;
  params.camera_to_target_guess = guess;
  params.correspondences = rct_image_tools::getCorrespondenceSet(*maybe_obs, target.points);

  rct_optimizations::PnPResult pnp_result = rct_optimizations::optimize(params);

  rct_ros_tools::printOptResults(pnp_result.converged, pnp_result.initial_cost_per_obs, pnp_result.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTransform(pnp_result.camera_to_target, "Camera", "Target", "RCT CAMERA TO TARGET");
  rct_ros_tools::printNewLine();

  return 0;
}
