/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <rct_image_tools/modified_circle_grid_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/pnp.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_loaders.h>
#include <rct_ros_tools/print_utils.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

static Eigen::Isometry3d solveCVPnP(const CameraIntrinsics& intr,
                                    const Correspondence2D3D::Set& correspondences)
{
  cv::Mat cam_matrix (3, 3, cv::DataType<double>::type);
  cv::setIdentity(cam_matrix);

  cam_matrix.at<double>(0, 0) = intr.fx();
  cam_matrix.at<double>(1, 1) = intr.fy();
  cam_matrix.at<double>(0, 2) = intr.cx();
  cam_matrix.at<double>(1, 2) = intr.cy();

  printCameraIntrinsics(intr.values, "Camera Intrinsics");
  printNewLine();

  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> target_points;
  image_points.reserve(correspondences.size());
  target_points.reserve(correspondences.size());
  for (const Correspondence2D3D& corr : correspondences)
  {
    image_points.push_back(cv::Point2d(corr.in_image.x(), corr.in_image.y()));
    target_points.push_back(cv::Point3d(corr.in_target.x(), corr.in_target.y(), corr.in_target.z()));
  }

  cv::Mat rvec (3, 1, cv::DataType<double>::type);
  cv::Mat tvec (3, 1, cv::DataType<double>::type);
  cv::solvePnP(target_points, image_points, cam_matrix, cv::noArray(), rvec, tvec);

  Eigen::Vector3d rr (Eigen::Vector3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)));
  Eigen::Isometry3d result(Eigen::AngleAxisd(rr.norm(), rr.normalized()));
  result.translation() = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

  printTransform(result, "Camera", "Target", "OpenCV solvePNP");
  printNewLine();

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "solve_pnp_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  try
  {
    // Load the image
    std::string image_path;
    if (!pnh.getParam("image_path", image_path))
    {
      ROS_ERROR_STREAM("Must set the 'image_path' private parameter");
      return 1;
    }
    cv::Mat mat = cv::imread(image_path);

    // Load target definition from parameter server
    ModifiedCircleGridTarget target;
    if (!TargetLoader<ModifiedCircleGridTarget>::load(pnh, "target_definition", target))
    {
      ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
      return 1;
    }

    // Load the camera intrinsics from the parameter server
    CameraIntrinsics intr;
    if (!loadIntrinsics(pnh, "intrinsics", intr))
    {
      ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
      return 1;
    }

    ModifiedCircleGridTargetFinder finder(target);
    rct_image_tools::TargetFeatures target_features = finder.findTargetFeatures(mat);

    cv::Mat show = finder.drawTargetFeatures(mat, target_features);
    cv::imshow("win", show);
    cv::waitKey();

    // Solve with OpenCV
    solveCVPnP(intr, target.createCorrespondences(target_features));

    // Solve with some native RCT function (for learning)
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess = guess * Eigen::Translation3d(0, 0, 0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    PnPProblem params;
    params.intr = intr;
    params.camera_to_target_guess = guess;
    params.correspondences = target.createCorrespondences(target_features);

    PnPResult pnp_result = optimize(params);

    printOptResults(pnp_result.converged, pnp_result.initial_cost_per_obs, pnp_result.final_cost_per_obs);
    printNewLine();

    printTransform(pnp_result.camera_to_target, "Camera", "Target", "RCT CAMERA TO TARGET");
    printNewLine();

  }
  catch(const std::exception& ex)
  {

  }


  return 0;
}
