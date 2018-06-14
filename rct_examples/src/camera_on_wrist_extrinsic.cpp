#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/extrinsic_camera_on_wrist.h>
#include "rct_examples/data_set.h"
#include "rct_examples/parameter_loaders.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

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

  // Load target definition from parameter server
  rct_image_tools::ModifiedCircleGridTarget target(10, 10, 0.0254);
  if (!rct_examples::loadTarget(pnh, "target_definition", target))
  {
    ROS_WARN_STREAM("Unable to load target from the 'target_definition' parameter struct");
  }

  // Load the camera intrinsics from the parameter server
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 510.0;
  intr.fy() = 510.0;
  intr.cx() = 320.2;
  intr.cy() = 208.9;
  if (!rct_examples::loadIntrinsics(pnh, "intrinsics", intr))
  {
    ROS_WARN_STREAM("Unable to load camera intrinsics from the 'intrinsics' parameter struct");
  }

  boost::optional<rct_examples::ExtrinsicDataSet> maybe_data_set = rct_examples::parseFromFile(data_path);
  if (!maybe_data_set)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
    return 2;
  }
  auto& data_set = *maybe_data_set;



  rct_image_tools::ImageObservationFinder obs_finder(target);

  rct_optimizations::ExtrinsicCameraOnWristProblem problem_def;
  problem_def.intr = intr;
  problem_def.wrist_to_camera_guess =
      rct_optimizations::poseCalToEigen(rct_optimizations::Pose6d({0, 0, -M_PI_2, 0.019, 0, 0.15}));

  problem_def.base_to_target_guess =
      rct_optimizations::poseCalToEigen(rct_optimizations::Pose6d({0, 0, -M_PI_2, 0.75, 0, 0}));

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

    // We got observations, let's process
    problem_def.wrist_poses.push_back(data_set.tool_poses[i]);

    rct_optimizations::ObservationSet obs_set;

    assert(maybe_obs->size() == target.points.size());
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      rct_optimizations::ObservationPair pair;
      pair.in_image = maybe_obs->at(j);
      pair.in_target = target.points[j];

      obs_set.push_back(pair);
    }

    problem_def.image_observations.push_back(obs_set);
  }

  // Run optimization
  auto opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  auto c = opt_result.wrist_to_camera;
  auto t = opt_result.base_to_target;

  std::cout << "Wrist To Camera:\n";
  std::cout << c.matrix() << "\n";
  std::cout << "Base to Target:\n";
  std::cout << t.matrix() << "\n";

  return 0;
}
