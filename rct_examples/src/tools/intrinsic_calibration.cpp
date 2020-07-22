#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/camera_intrinsic.h>
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/print_utils.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/calib3d.hpp>

void opencvCameraCalibration(const std::vector<rct_optimizations::Correspondence2D3D::Set>& obs,
                             const cv::Size& image_size,
                             const rct_optimizations::CameraIntrinsics& intr)
{
  std::vector<std::vector<cv::Vec3f>> object_points;
  std::vector<std::vector<cv::Vec2f>> image_points;

  for (const auto& o : obs)
  {
    std::vector<cv::Vec3f> op;
    std::vector<cv::Vec2f> ip;

    for (const auto& pair : o)
    {
      op.push_back(cv::Vec3f(pair.in_target(0), pair.in_target(1), pair.in_target(2)));
      ip.push_back(cv::Vec2f(pair.in_image(0), pair.in_image(1)));
    }

    object_points.push_back(op);
    image_points.push_back(ip);
  }

  cv::Mat camera_matrix (3, 3, cv::DataType<double>::type);
  cv::setIdentity(camera_matrix);
  camera_matrix.at<double>(0, 0) = intr.fx();
  camera_matrix.at<double>(1, 1) = intr.fy();
  camera_matrix.at<double>(0, 2) = intr.cx();
  camera_matrix.at<double>(1, 2) = intr.cy();
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

  std::array<double, 4> intr_values;
  intr_values[0] = camera_matrix.at<double>(0, 0);
  intr_values[1] = camera_matrix.at<double>(1, 1);
  intr_values[2] = camera_matrix.at<double>(0, 2);
  intr_values[3] = camera_matrix.at<double>(2, 2);
  rct_ros_tools::printCameraIntrinsics(intr_values, "OpenCV Intrinsics");
  rct_ros_tools::printNewLine();

  std::array<double, 5> dist_values;
  dist_values[0] = dist_coeffs.at<double>(0);
  dist_values[1] = dist_coeffs.at<double>(1);
  dist_values[2] = dist_coeffs.at<double>(2);
  dist_values[3] = dist_coeffs.at<double>(3);
  dist_values[4] = dist_coeffs.at<double>(4);
  rct_ros_tools::printCameraDistortion(dist_values, "OpenCV Distortion");
  rct_ros_tools::printNewLine();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  // Load Image Set
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  auto& data_set = *maybe_data_set;

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

  // Create obs finder
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder (target);

  // Construct problem
  rct_optimizations::IntrinsicEstimationProblem problem_def;
  problem_def.intrinsics_guess = intr;

  for (std::size_t i = 0; i < data_set.images.size(); ++i)
  {
    // Extract observations
    auto maybe_obs = obs_finder.findObservations(data_set.images[i]);
    if (!maybe_obs)
    {
      continue;
    }

    // Show drawing
    cv::imshow("points", obs_finder.drawObservations(data_set.images[i], *maybe_obs));
    cv::waitKey();

    problem_def.image_observations.push_back(rct_image_tools::getCorrespondenceSet(*maybe_obs, target.points));
  }

  // Run optimization
  auto opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  rct_ros_tools::printTitle("Calibration Complete");

  rct_ros_tools::printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  auto new_intr = opt_result.intrinsics;
  auto new_dist = opt_result.distortions;

  rct_ros_tools::printCameraIntrinsics(new_intr.values, "RCT Intrinsics");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printCameraDistortion(new_dist, "RCT Distortion");
  rct_ros_tools::printNewLine();

  std::cout << opt_result.covariance.toString() << std::endl;

  // Also try the OpenCV cameraCalibrate function
  rct_ros_tools::printTitle("OpenCV Calibration");
  opencvCameraCalibration(problem_def.image_observations, data_set.images.front().size(),
                          problem_def.intrinsics_guess);
  return 0;
}
