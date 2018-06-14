/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/experimental/pnp.h>
#include <rct_examples/parameter_loaders.h>

static Eigen::Affine3d solveCVPnP(const rct_optimizations::CameraIntrinsics& intr,
                                  const rct_image_tools::ModifiedCircleGridTarget& target,
                                  const std::vector<Eigen::Vector2d>& obs)
{
  cv::Mat cam_matrix (3, 3, cv::DataType<double>::type);
  cv::setIdentity(cam_matrix);

  cam_matrix.at<double>(0, 0) = intr.fx();
  cam_matrix.at<double>(1, 1) = intr.fy();
  cam_matrix.at<double>(0, 2) = intr.cx();
  cam_matrix.at<double>(1, 2) = intr.cy();

  std::cout << "Camera matrix:\n" << cam_matrix << "\n";

  std::vector<cv::Point2d> image_points;
  for (const auto o : obs)
    image_points.push_back(cv::Point2d(o(0), o(1)));

  std::vector<cv::Point3d> target_points;
  for (const auto p : target.points)
    target_points.push_back( cv::Point3d(p(0), p(1), p(2)) );

  cv::Mat rvec (3, 1, cv::DataType<double>::type);
  cv::Mat tvec (3, 1, cv::DataType<double>::type);
  cv::solvePnP(target_points, image_points, cam_matrix, cv::noArray(), rvec, tvec);
  std::cout << "rvec: " << rvec << "\n";
  std::cout << "tvec: " << tvec << "\n";

  Eigen::Affine3d result;
  result.setIdentity();
  result.translation() = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

  Eigen::Vector3d rr (Eigen::Vector3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)));
  std::cout << "RR " << rr << "\n";

  Eigen::AngleAxisd rot (rr.norm(), rr.normalized());

  result.linear() = rot.toRotationMatrix();
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
  rct_image_tools::ModifiedCircleGridTarget target(5, 5, 0.015);
  if (!rct_examples::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
  }

  // Load the camera intrinsics from the parameter server
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 1411.0;
  intr.fy() = 1408.0;
  intr.cx() = 807.2;
  intr.cy() = 615.0;
  if (!rct_examples::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
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
  Eigen::Affine3d cv_pose = solveCVPnP(intr, target, *maybe_obs);
  std::cout << "CV_POSE\n" << cv_pose.matrix() << "\n";

  // Solve with some native RCT function (for learning)
  Eigen::Affine3d guess = Eigen::Affine3d::Identity();
  guess = guess * Eigen::Translation3d(0,0,0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  rct_optimizations::PnPProblem params;
  params.intr = intr;
  params.camera_to_target_guess = guess;

  for (std::size_t i = 0; i < maybe_obs->size(); ++i)
  {
    rct_optimizations::Correspondence2D3D pair;
    pair.in_image = (*maybe_obs)[i];
    pair.in_target = target.points[i];
    params.correspondences.push_back(pair);
  }

  rct_optimizations::PnPResult pnp_result = rct_optimizations::optimize(params);
  std::cout << "RCT_POSE\n" << pnp_result.camera_to_target.matrix() << "\n";

  return 0;
}
