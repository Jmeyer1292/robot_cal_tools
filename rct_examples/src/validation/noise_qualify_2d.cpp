#include <Eigen/Dense>
#include <rct_optimizations/validation/noise_qualifier.h>
#include <rct_optimizations/types.h>
#include <rct_image_tools/image_observation_finder.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.hpp>
#include <ros/ros.h>

#include <rct_optimizations/pnp.h>
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/data_set.h"

int main(int argc, char** argv)
{
  ros::NodeHandle pnh("~");

  // Parse parameters
  std::string path;
  if (!pnh.getParam("data_path", path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  int rows;
  if (!pnh.getParam("rows", rows))
  {
    ROS_ERROR("Must set 'rows' parameter");
    return 1;
  }

  int cols;
  if (!pnh.getParam("columns", cols))
  {
    ROS_ERROR("Must set 'columns' parameter");
    return 1;
  }

  //make target
  rct_image_tools::ModifiedCircleGridTarget target (rows, cols, 0.01);
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  YAML::Node root = YAML::LoadFile(path + std::string("/data.yaml"));

  //Load camera intrinsics
  rct_optimizations::CameraIntrinsics camera = rct_ros_tools::loadIntrinsics(path +  std::string("/camera_data.yaml"));

  //reserve observations
  std::vector<rct_optimizations::PnPProblem> problem_set;
  problem_set.reserve(root.size());

  //create observations
  for (std::size_t i = 0; i < root.size(); ++i)
  {
    rct_optimizations::PnPProblem problem;
    problem.intr = camera;

    const auto pose_path = root[i]["pose"].as<std::string>();
    problem.camera_to_target_guess = rct_ros_tools::loadPose(path + pose_path);

    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const std::string image_name = path + img_path;
    static cv::Mat image = rct_ros_tools::readImageOpenCV(image_name);

    auto maybe_obs = obs_finder.findObservations(image);
    if (!maybe_obs)
    {
      cv::imshow("points", image);
      cv::waitKey();
      continue;
    }

    // And loop over each detected dot:
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      problem.correspondences[j].in_image = maybe_obs->at(j); // The obs finder and target define their points in the same order!
      problem.correspondences[j].in_target = target.points[j];
    };
  }

  //start with a good guess

  rct_optimizations::PnPNoiseStat output = rct_optimizations::qualifyNoise2D(problem_set);


  ROS_INFO_STREAM("The standard deviation is : x: " << output.x.std_dev <<
                                           "\n y: " << output.y.std_dev <<
                                           "\n z: " << output.z.std_dev <<
                                           "\n i: " << output.q.qx.std_dev <<
                                           "\n j: " << output.q.qy.std_dev <<
                                           "\n k: " << output.q.qz.std_dev <<
                                           "\n w: " << output.q.qw.std_dev <<
                                                                      "\n");
}
