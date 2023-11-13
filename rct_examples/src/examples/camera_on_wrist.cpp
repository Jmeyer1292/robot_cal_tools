/***
 * This file contains a standalone example of performing a calibration to understand the transform
 * between a robot wrist and the camera attached to it. It assumes that we have:
 *  1. The camera intrinsics
 *  2. The target size and spacing
 *  3. A bunch of pictures of the target, and
 *  4. The position of the robot wrist (in its base frame) when each of the pictures was taken
 *
 * This is only supposed to be a sample to ease understanding. See the equivalent tool for a
 * configurable offline processor of your data
 */

// This include is used to load images and the associated wrist poses from the data/ directory
// of this package. It's my only concession to the "self-contained rule".
#include <rct_ros_tools/data_set.h>
// This include provide useful print functions for outputing results to terminal
#include <rct_common/print_utils.h>
// This header brings in a tool for finding the target in a given image
#include <rct_image_tools/modified_circle_grid_finder.h>
// This header brings in he calibration function for 'moving camera' on robot wrist - what we
// want to calibrate here.
#include <rct_optimizations/extrinsic_hand_eye.h>
// This header brings in the ros::package::getPath() command so I can find the data set
// on your computer.
#include <ros/package.h>
// For std::cout
#include <iostream>

using namespace rct_common;

int extrinsicWristCameraCalibration()
{
  // The general goal is to fill out the 'Extrinsic Camera on Wrist' problem definition
  // and then call 'rct_optimizations::optimize()' on it.

  // Step 1: Create a problem definition. It requires we set:
  // - Camera Intrinsics
  // - Guesses for the 'base to target' and 'wrist to camera'
  // - The wrist poses
  // - Correspondences (2D in image to 3D in target) for each wrist pose
  rct_optimizations::ExtrinsicHandEyeProblem2D3D problem_def;

  // Step 1: Define the camera intrinsics - I know mine so I write them here. If you don't know
  // yours, use the opencv calibration tool to figure them out!
  problem_def.intr.fx() = 510.0;
  problem_def.intr.fy() = 510.0;
  problem_def.intr.cx() = 320.2;
  problem_def.intr.cy() = 208.9;

  // Step 2: Let's add the guesses next. See the rct_examples/README.md for a discussion about the
  // reference frames for the camera and target!
  Eigen::Vector3d wrist_to_camera_tx(0.015, 0, 0.15);  // Guess for wrist-to-camera
  Eigen::Matrix3d wrist_to_camera_rot;
  wrist_to_camera_rot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  problem_def.camera_mount_to_camera_guess.translation() = wrist_to_camera_tx;
  problem_def.camera_mount_to_camera_guess.linear() = wrist_to_camera_rot;

  Eigen::Vector3d base_to_target_tx(1, 0, 0);  // Guess for base-to-target
  Eigen::Matrix3d base_to_target_rot;
  base_to_target_rot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  problem_def.target_mount_to_target_guess.translation() = base_to_target_tx;
  problem_def.target_mount_to_target_guess.linear() = base_to_target_rot;

  // Step 3: We need to specify the wrist-poses and the correspondences. I'm going to use a simple
  // tool I wrote to load images and wrist poses from a file. You need to implement the data
  // collection logic for your application.

  // Step 3.1: Load the data set (and make sure it worked)
  const std::string data_path = ros::package::getPath("rct_examples") + "/data/test_set_10x10/cal_data.yaml";
  boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(data_path);
  // Attempt to load the data set via the data record yaml file:
  if (!maybe_data_set)
  {
    std::cerr << "Failed to parse data set from path = " << data_path << "\n";
    return 1;
  }
  // Now that its loaded let's create some aliases to make this nicer
  const std::vector<cv::Mat>& image_set = maybe_data_set->images;
  const std::vector<Eigen::Isometry3d>& wrist_poses = maybe_data_set->tool_poses;

  // Step 3.2: We need to conver the images of calibration targets into sets of correspondences.
  // In our case, each dot on the target becomes a correspondence: A pair of positions, one for
  // where the dot was seen in the image (u, v in pixels) and one for where that same dot is in the
  // target (x,y,z in meters). Repeat for each image.

  // The 'rct_image_tools' package provides a class for describing the target (it's 3D points) and
  // a class for finding that target in an image. So let's create a target...
  rct_image_tools::ModifiedCircleGridTarget target(10, 10, 0.0254);  // rows, cols, spacing between dots

  // The Observation Finder class uses the target definition to search images
  rct_image_tools::ModifiedCircleGridTargetFinder target_finder(target);

  // Now we'll loop over the images and 1) find the dots, 2) pair them with the target location,
  // and 3) push them into the problem definition.
  // This process of finding the dots can fail, we we make sure they are found before adding the
  // wrist location for that image.
  problem_def.observations.reserve(image_set.size());
  for (std::size_t i = 0; i < image_set.size(); ++i)
  {
    // Try to find the circle grid in this image:
    rct_image_tools::TargetFeatures target_features;
    try
    {
      target_features = target_finder.findTargetFeatures(image_set[i]);
    }
    catch (const std::exception& ex)
    {
      std::cerr << "Unable to find the circle grid in image: " << i << "\n";
      continue;
    }

    // We found the target! Let's "zip-up" the correspondence pairs for this image - one for each
    // dot. So we create a data structure:
    rct_optimizations::Observation2D3D obs;
    obs.correspondence_set.reserve(target_features.size());

    // Create correspondences given the target and the features in the image
    obs.correspondence_set = target.createCorrespondences(target_features);

    // Let's add the wrist pose for this image as the "to_camera_mount" transform
    // Since the target is not moving relative to the camera, set the "to_target_mount" transform to identity
    obs.to_camera_mount = wrist_poses[i];
    obs.to_target_mount = Eigen::Isometry3d::Identity();

    // Finally let's add this observation to the problem!
    problem_def.observations.push_back(obs);
  }

  // Step 4: You defined a problem, let the tools solve it! Call 'optimize()'.
  rct_optimizations::ExtrinsicHandEyeResult opt_result = rct_optimizations::optimize(problem_def);

  // Step 5: Do something with your results. Here I just print the results, but you might want to
  // update a data structure, save to a file, push to a mutable joint or mutable state publisher in
  // ROS. The options are many, and it's up to you. We just try to help solve the problem.
  printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
  printNewLine();

  // Note: Convergence and low cost does not mean a good calibration. See the calibration primer
  // readme on the main page of this repo.
  Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
  printTransform(c, "Wrist", "Camera", "WRIST TO CAMERA");
  printNewLine();

  Eigen::Isometry3d t = opt_result.target_mount_to_target;
  printTransform(t, "Base", "Target", "BASE TO TARGET");
  printNewLine();

  return 0;
}

#ifndef RCT_ENABLE_TESTING

int main() { return extrinsicWristCameraCalibration(); }

#else

#include <gtest/gtest.h>

TEST(ExtrinsicCamera, ExtrinsicCamera) { EXPECT_EQ(extrinsicWristCameraCalibration(), 0); }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
