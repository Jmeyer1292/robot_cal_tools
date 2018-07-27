#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/rail_intrinsic.h>
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/print_utils.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

// This cost has a custom file loader & format
struct RailCalibrationScene
{
  std::vector<double> rail_poses;
  std::vector<rct_optimizations::CorrespondenceSet> correspondences;
};

struct RailCalibrationData
{
  std::vector<RailCalibrationScene> scenes;
};

bool parseNameData(const std::string& name, int& scene_id, int& position_id)
{
  return true;
}

RailCalibrationData parseCalData(const std::string& base_path, const std::vector<std::string>& images,
                                 const rct_image_tools::ModifiedCircleGridTarget& target)
{
  std::map<int, RailCalibrationScene> scenes;
  rct_image_tools::ModifiedCircleGridObservationFinder finder (target);

  for (const auto& f : images)
  {
    const auto full_path = base_path + f;

    cv::Mat image = cv::imread(full_path);
    if (image.data == nullptr) {
      ROS_WARN_STREAM("Unable to load " << full_path);
      continue;
    }

    int scene_id = -1;
    int position_id = -1;

    if (!parseNameData(f, scene_id, position_id)) {
      ROS_WARN_STREAM("Unable to parse name: " << f);
      continue;
    }

    cv::imshow("in", image);
    auto maybe = finder.findObservations(image);
    if (maybe)
    {
      cv::imshow("found", finder.drawObservations(image, *maybe));
    }

    cv::waitKey(0);
  }

  return {};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rct_rail_intrinsic_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  rct_image_tools::ModifiedCircleGridTarget target;
  if (!rct_ros_tools::loadTarget(pnh, "target", target))
  {
    return 1;
  }

  rct_optimizations::CameraIntrinsics intr;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    return 2;
  }

  // Load data sets
  std::vector<std::string> image_files;
  if (!pnh.getParam("images", image_files))
  {
    return 3;
  }

  std::string base_path;
  if (!pnh.getParam("image_base", base_path))
  {
    return 4;
  }

  auto data = parseCalData(base_path, image_files, target);

  // Now we construct our problem
  rct_optimizations::RailIntrinsicProblem problem;
  problem.intrinsics_guess = intr;
  problem.skew_x_guess = problem.skew_y_guess = 0.0;

  for (const auto& scene : data.scenes)
  {
    // set the target pose guess
    auto target_pose = Eigen::Affine3d::Identity();
    target_pose.translation() = Eigen::Vector3d(0, 0, 1.0);
    target_pose.matrix().col(0).head<3>() = Eigen::Vector3d(1, 0, 0);
    target_pose.matrix().col(0).head<3>() = Eigen::Vector3d(0, 1, 0);
    target_pose.matrix().col(0).head<3>() = Eigen::Vector3d(0, 0, 1);
    problem.extrinsic_guesses.push_back(target_pose);

    problem.image_observations.push_back(scene.correspondences);
    problem.rail_distances.push_back(scene.rail_poses);
  }

  auto result = rct_optimizations::optimize(problem);

  // Print the results
  rct_ros_tools::printOptResults(result.converged, result.initial_cost_per_obs, result.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  rct_ros_tools::printCameraIntrinsics(result.intrinsics.values, "RCT Intrinsics");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printCameraDistortion(result.distortions, "RCT Distortion");
  rct_ros_tools::printNewLine();

  for (std::size_t i = 0; i < result.target_transforms.size(); ++i)
  {
    std::cout << "Target Origin Scene " << i << "\n";
    std::cout << result.target_transforms[i].matrix() << "\n";
  }

  std::cout << "Skew: " << result.skew_x << ", " << result.skew_y << "\n";

  return 0;
}
