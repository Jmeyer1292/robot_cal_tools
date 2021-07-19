// Utilities for loading data sets and calib parameters from YAML files via ROS
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/print_utils.h>
// To find 2D  observations from images
#include <rct_image_tools/image_utils.h>
// The calibration function for 'moving camera' on robot wrist
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/pnp.h>
#include <rct_optimizations/serialization/problems.h>
#include <rct_optimizations/validation/homography_validation.h>

// For display of found targets
#include <opencv2/highgui/highgui.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <rct_optimizations/ceres_math_utilities.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

const std::string WINDOW = "window";

static void reproject(const Eigen::Isometry3d& wrist_to_camera, const Eigen::Isometry3d& base_to_target,
                      const Observation2D3D& obs, const CameraIntrinsics& intr, const cv::Mat& image)
{
  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  Eigen::Isometry3d camera_to_target = (obs.to_camera_mount * wrist_to_camera).inverse() * base_to_target;

  std::vector<Eigen::Vector3d> target_points;
  target_points.reserve(obs.correspondence_set.size());
  std::transform(obs.correspondence_set.begin(), obs.correspondence_set.end(), std::back_inserter(target_points),
                 [](const rct_optimizations::Correspondence2D3D& corr) { return corr.in_target; });
  std::vector<cv::Point2d> reprojections = getReprojections(camera_to_target, intr, target_points);

  cv::Mat frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);

  PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = obs.correspondence_set;
  pb.intr = intr;
  PnPResult r = optimize(pb);

  printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  printNewLine();

  printTransform(r.camera_to_target, "Camera", "Target", "PNP");
  printNewLine();

  printTransformDiff(camera_to_target, r.camera_to_target, "Camera", "Target", "PNP DIFF");
  printNewLine();

  cv::imshow(WINDOW, frame);
  cv::waitKey();
}

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

  // Load the data set path from ROS param
  try
  {
    double homography_threshold = get<double>(pnh, "homography_threshold");

    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Attempt to load the data set via the data record yaml file:
    std::string data_path = get<std::string>(pnh, "data_path");
    boost::optional<ExtrinsicDataSet> maybe_data_set = parseFromFile(data_path);
    if (!maybe_data_set)
      throw std::runtime_error("Failed to parse data set from path = " + data_path);

    // We know it exists, so define a helpful alias
    const ExtrinsicDataSet& data_set = *maybe_data_set;

    // Lets create a class that will search for the target in our raw images.
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(target_finder_config);

    // Now we create our calibration problem
    ExtrinsicHandEyeProblem2D3D problem;
    problem.intr = intr;  // Set the camera properties

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    problem.target_mount_to_target_guess = loadPose(pnh, "base_to_target_guess");
    problem.camera_mount_to_camera_guess = loadPose(pnh, "wrist_to_camera_guess");

    // Create a named OpenCV window for viewing the images
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    problem.observations.reserve(data_set.images.size());

    // The target may not be identified in all images, so let's keep track the indices of the images for which the
    // target was identified
    std::vector<std::size_t> valid_image_indices;
    valid_image_indices.reserve(data_set.images.size());

    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // For each image we need to:
      //// 1. Try to find the target features in this image:
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder->findTargetFeatures(data_set.images[i]);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features");
        ROS_INFO_STREAM("Found " << target_features.size() << " target features");

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

      Observation2D3D obs;
      obs.correspondence_set.reserve(target_features.size());

      //// 2. Record the wrist position and target features
      obs.to_camera_mount = data_set.tool_poses[i];
      obs.to_target_mount = Eigen::Isometry3d::Identity();
      obs.correspondence_set = target_finder->target().createCorrespondences(target_features);

      //// 3. Check that a homography matrix can accurately reproject the observed points onto the expected target points within a defined threshold
      rct_optimizations::RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(),
                                                                    obs.correspondence_set.size() / 3);
      Eigen::VectorXd homography_error =
          rct_optimizations::calculateHomographyError(obs.correspondence_set, random_sampler);
      if (homography_error.array().mean() > homography_threshold)
      {
        ROS_ERROR_STREAM("Homography error exceeds threshold (" << homography_error.array().mean() << ")");
        continue;
      }

      //// 3. And finally add that to the problem
      problem.observations.push_back(obs);
      valid_image_indices.push_back(i);
    }

    // Now we have a defined problem, run optimization:
    ExtrinsicHandEyeResult opt_result = optimize(problem);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
    printTransform(c, "Wrist", "Camera", "WRIST TO CAMERA");
    printNewLine();

    Eigen::Isometry3d t = opt_result.target_mount_to_target;
    printTransform(t, "Base", "Target", "BASE TO TARGET");
    printNewLine();

    std::cout << opt_result.covariance.toString() << std::endl;

    printTitle("REPROJECTION ERROR");
    for (std::size_t i = 0; i < problem.observations.size(); ++i)
    {
      printTitle("REPROJECT OBSERVATION " + std::to_string(i));
      std::size_t image_idx = valid_image_indices[i];
      reproject(opt_result.camera_mount_to_camera, opt_result.target_mount_to_target, problem.observations[i], intr,
                data_set.images[image_idx]);
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
