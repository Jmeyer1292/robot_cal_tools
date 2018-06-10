#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/camera_intrinsic.h>
#include "rct_examples/data_set.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/calib3d.hpp>

void opencvCal()
{
//  cv::calibrateCamera()
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

  // Process each image into observations
  // Load Target Definition
  rct_image_tools::ModifiedCircleGridTarget target(5, 5, 0.015);

  rct_image_tools::ImageObservationFinder obs_finder(target);

  // Construct problem
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 1400.0;
  intr.fy() = 1400.0;
  intr.cx() = 800.;
  intr.cy() = 600.;

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

  auto new_intr = opt_result.intrinsics;
  auto new_dist = opt_result.distortions;

  std::cout << "New Intr:\nfx = " << new_intr.fx() << "\tfy = " << new_intr.fy() << "\ncx = " << new_intr.cx()
            << "\tcy = " << new_intr.cy() << "\n";

  std::cout << "Distortions:\n";
  std::cout << new_dist[0] << " " << new_dist[1] << " " << new_dist[2] << " " << new_dist[3] << " " << new_dist[4] << "\n";

  return 0;
}
