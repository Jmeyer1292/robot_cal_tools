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
  if (argc != 2)
  {
    std::cout << "Usage: ./solve_pnp image_path\n";
    return 1;
  }

  // Load the image
  std::string image_path (argv[1]);
  cv::Mat mat = cv::imread(image_path);

  // Load the target
  rct_image_tools::ModifiedCircleGridTarget target (5, 5, 0.015);
  rct_image_tools::ImageObservationFinder finder (target);

  auto maybe_obs = finder.findObservations(mat);
  if (!maybe_obs)
  {
    std::cerr << "Unable to detect the target!\n";
    return 1;
  }

  cv::Mat show = finder.drawObservations(mat, *maybe_obs);
  cv::imshow("win", show);
  cv::waitKey();

  // Load the camera
  rct_optimizations::CameraIntrinsics intr;
  intr.cx() = 800;
  intr.cy() = 600;
  intr.fx() = 1400;
  intr.fy() = 1400;

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
    rct_optimizations::ObservationPair pair;
    pair.in_image = (*maybe_obs)[i];
    pair.in_target = target.points[i];
    params.correspondences.push_back(pair);
  }

  rct_optimizations::PnPResult pnp_result = rct_optimizations::optimize(params);
  std::cout << "RCT_POSE\n" << pnp_result.camera_to_target.matrix() << "\n";

  return 0;
}
