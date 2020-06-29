#include <opencv/cv.hpp>
#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/validation/noise_qualification.h>
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

template<typename T>
bool get(const ros::NodeHandle &nh, const std::string &key, T &val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noise_qualification_2d");
  ros::NodeHandle pnh("~");

  // Parse parameters
  std::string data_path;
  if (!get(pnh, "data_path", data_path))
    return -1;

  std::string data_file;
  if (!get(pnh, "data_file", data_file))
    return -1;

  try
  {
    // Load the target definition and observation finder
    rct_image_tools::ModifiedCircleGridTarget target = rct_ros_tools::loadTarget(pnh, "target_definition");
    rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

    // Load camera intrinsics
    rct_optimizations::CameraIntrinsics camera = rct_ros_tools::loadIntrinsics(pnh, "intrinsics");

    // Load an initial guess for the camera to target transformation
    Eigen::Isometry3d camera_to_target_guess = rct_ros_tools::loadPose(pnh, "camera_to_target_guess");

    // Load the data file which specifies the location of the images on which to perform the noise qualification
    YAML::Node root = YAML::LoadFile(data_file);

    // Set up the noise qualification inputs
    std::vector<rct_optimizations::PnPProblem> problem_set;
    problem_set.reserve(root.size());
    for (std::size_t i = 0; i < root.size(); ++i)
    {
      // Each entry should have an image path. This path is relative to the root_path directory!
      const auto img_path = root[i]["image"].as<std::string>();
      const std::string image_name = data_path + "/" + img_path;
      static cv::Mat image = rct_ros_tools::readImageOpenCV(image_name);

      // Find the observations in the image
      auto maybe_obs = obs_finder.findObservations(image);
      if (!maybe_obs)
      {
        cv::imshow("points", image);
        ROS_INFO_STREAM("Hit enter in the OpenCV window to continue");
        cv::waitKey();
        continue;
      }
      else
      {
        // Show the points we detected
        cv::imshow("points", obs_finder.drawObservations(image, *maybe_obs));
        ROS_INFO_STREAM("Hit enter in the OpenCV window to continue");
        cv::waitKey();
      }

      // Set up the PnP problem for this image
      rct_optimizations::PnPProblem problem;
      problem.intr = camera;
      problem.camera_to_target_guess = camera_to_target_guess;

      // Add the detected correspondences
      problem.correspondences.reserve(maybe_obs->size());
      for (std::size_t j = 0; j < maybe_obs->size(); ++j)
      {
        problem.correspondences.emplace_back(maybe_obs->at(j), target.points.at(j));
      };

      problem_set.push_back(problem);
    }

    // Perform the noise qualification
    rct_optimizations::PnPNoiseStat result = rct_optimizations::qualifyNoise2D(problem_set);

    // Print the results
    Eigen::IOFormat fmt(4, 0, ",", "\n", "[", "]");
    ROS_INFO_STREAM("Camera to Target Noise Results");
    ROS_INFO_STREAM("Position mean (m)\n" << result.p_stat.mean.transpose().format(fmt));
    ROS_INFO_STREAM("Position standard deviation (m)\n" << result.p_stat.stdev.transpose().format(fmt));
    ROS_INFO_STREAM("Quaternion mean (qx, qy, qz, qw)\n" << result.q_stat.mean.coeffs().transpose().format(fmt));
    ROS_INFO_STREAM("Quaternion standard deviation\n" << result.q_stat.stdev);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
