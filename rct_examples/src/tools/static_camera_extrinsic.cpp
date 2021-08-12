// Utilities for loading data sets and calib parameters from YAML files via ROS
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/print_utils.h>
#include <rct_ros_tools/loader_utils.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/validation/homography_validation.h>
// Calibration analysis
#include "hand_eye_calibration_analysis.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

std::string WINDOW = "window";

template <typename T>
T get(const ros::NodeHandle& nh, const std::string& key)
{
  T val;
  if (!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  try
  {
    double homography_threshold = get<double>(pnh, "homography_threshold");

    // Load the data set path from ROS param
    std::string data_path = get<std::string>(pnh, "data_path");

    // Attempt to load the data set from the specified path
    boost::optional<ExtrinsicDataSet> maybe_data_set = parseFromFile(data_path);
    if (!maybe_data_set)
      throw std::runtime_error("Failed to parse data set from path = " + data_path);

    // We know it exists, so define a helpful alias
    const ExtrinsicDataSet& data_set = *maybe_data_set;

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(toYAML(target_finder_config));

    // Load the camera intrinsic parameters
    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Create a named OpenCV window for viewing the images
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

    // Now we construct our problem:
    ExtrinsicHandEyeProblem2D3D problem;
    // Our camera intrinsics to use
    problem.intr = intr;

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    problem.camera_mount_to_camera_guess = loadPose(pnh, "base_to_camera_guess");
    problem.target_mount_to_target_guess = loadPose(pnh, "wrist_to_target_guess");

    std::vector<cv::Mat> found_images;
    found_images.reserve(data_set.images.size());

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    problem.observations.reserve(data_set.images.size());
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the target features in this image:
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder->findTargetFeatures(data_set.images[i]);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features");
        ROS_INFO_STREAM("Found " << target_features.size() << " target features");

        Observation2D3D obs;
        obs.to_target_mount = data_set.tool_poses[i];
        obs.to_camera_mount = Eigen::Isometry3d::Identity();
        obs.correspondence_set = target_finder->target().createCorrespondences(target_features);

        // Check that a homography matrix can accurately reproject the observed points onto the expected target points within a defined threshold
        rct_optimizations::RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(),
                                                                      obs.correspondence_set.size() / 3);
        Eigen::VectorXd homography_error =
            rct_optimizations::calculateHomographyError(obs.correspondence_set, random_sampler);
        if (homography_error.array().mean() > homography_threshold)
          throw std::runtime_error("Homography error exceeds threshold (" + std::to_string(homography_error.array().mean()) + ")");

        // Add that to the problem
        problem.observations.push_back(obs);
        found_images.push_back(data_set.images[i]);

        // Show the points we detected
        cv::imshow(WINDOW, target_finder->drawTargetFeatures(data_set.images[i], target_features));
        cv::waitKey();
      }
      catch (const std::runtime_error& ex)
      {
        ROS_WARN_STREAM("Image " << i << ": '" << ex.what() << "'");
        cv::imshow(WINDOW, data_set.images[i]);
        cv::waitKey();
        continue;
      }
    }

    // Run optimization
    ExtrinsicHandEyeResult opt_result = optimize(problem);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
    printTransform(c, "Base", "Camera", "BASE TO CAMERA");
    printNewLine();

    Eigen::Isometry3d t = opt_result.target_mount_to_target;
    printTransform(t, "Wrist", "Target", "WRIST_TO_TARGET");
    printNewLine();

    std::cout << opt_result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

    // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
    // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
    // parameters We will then see how much this transform differs from the same transform calculated using the results
    // of the extrinsic calibration
    analyzeResults(problem, opt_result, found_images, WINDOW);
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
