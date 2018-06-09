/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */


#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <rct_optimizations/types.h>
#include <rct_image_tools/image_observation_finder.h>


#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <ceres/ceres.h>

namespace
{
struct CostFunc
{
public:
  CostFunc(const rct_optimizations::CameraIntrinsics& intr, const Eigen::Vector3d& pt_in_target,
           const Eigen::Vector2d& pt_in_image)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
  {}

  template<typename T>
  bool operator()(const T* const target_pose, T* const residual) const
  {
    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;


    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in camera coordinates
    rct_optimizations::transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    T xy_image[2];
    rct_optimizations::projectPoint(intr_, camera_point, xy_image);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

  rct_optimizations::CameraIntrinsics intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

}

static Eigen::Affine3d solvePnP(const rct_optimizations::CameraIntrinsics& intr, const rct_image_tools::ModifiedCircleGridTarget& target,
                                const std::vector<Eigen::Vector2d>& obs, const Eigen::Affine3d& guess = Eigen::Affine3d::Identity())
{
  using namespace rct_optimizations;
  Pose6d internal_camera_to_target = poseEigenToCal(guess);
  std::cout << "aaxis: " << internal_camera_to_target.rx() << " " << internal_camera_to_target.ry() << " " << internal_camera_to_target.rz() << "\n";

  ceres::Problem problem;

  for (std::size_t j = 0; j < obs.size(); ++j) // For each 3D point seen in the 2D image
  {
    // Define
    const auto& img_obs = obs[j];
    const auto& point_in_target = target.points[j];

    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto* cost_fn = new CostFunc(intr, point_in_target, img_obs);

    auto* cost_block = new ceres::AutoDiffCostFunction<CostFunc, 2, 6>(cost_fn);

    problem.AddResidualBlock(cost_block, NULL, internal_camera_to_target.values.data());
  }


  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);
  std::cout << "init cost: " << summary.initial_cost / summary.num_residuals << "\n";
  std::cout << "final cost: " << summary.final_cost / summary.num_residuals << "\n";

  return poseCalToEigen(internal_camera_to_target);
}

static Eigen::Affine3d solveCVPnP(const rct_optimizations::CameraIntrinsics& intr, const rct_image_tools::ModifiedCircleGridTarget& target,
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
  Eigen::Affine3d rct_pose = solvePnP(intr, target, *maybe_obs, guess);
  std::cout << "RCT_POSE\n" << rct_pose.matrix() << "\n";
}
