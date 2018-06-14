#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/camera_intrinsic.h>
#include "rct_examples/data_set.h"
#include "rct_examples/parameter_loaders.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/calib3d.hpp>

void opencvCameraCalibration(const std::vector<rct_optimizations::CorrespondenceSet>& obs,
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


  std::cout << "OpenCV Camera Matrix:\n" << camera_matrix << "\n";
  std::cout << "OpenCV Camera Distortions:\n" << dist_coeffs << "\n";
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

  boost::optional<rct_examples::ExtrinsicDataSet> maybe_data_set = rct_examples::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  auto& data_set = *maybe_data_set;

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

    rct_optimizations::CorrespondenceSet obs_set;

    assert(maybe_obs->size() == target.points.size());
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      rct_optimizations::Correspondence2D3D pair;
      pair.in_image = maybe_obs->at(j);
      pair.in_target = target.points[j];

      obs_set.push_back(pair);
    }

    problem_def.image_observations.push_back(obs_set);
  }

  // Run optimization
  auto opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "---Calibration Complete---\n";
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  auto new_intr = opt_result.intrinsics;
  auto new_dist = opt_result.distortions;

  std::cout << "New Intr:\nfx = " << new_intr.fx() << "\tfy = " << new_intr.fy() << "\ncx = " << new_intr.cx()
            << "\tcy = " << new_intr.cy() << "\n\n";

  std::cout << "Distortions:\n";
  std::cout << new_dist[0] << " " << new_dist[1] << " " << new_dist[2] << " " << new_dist[3] << " "
                           << new_dist[4] << "\n\n";

  // Also try the OpenCV cameraCalibrate function
  std::cout << "---OpenCV Calibration---\n";
  opencvCameraCalibration(problem_def.image_observations, data_set.images.front().size(),
                          problem_def.intrinsics_guess);
  return 0;
}
