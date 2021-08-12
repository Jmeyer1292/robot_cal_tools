// Utilities for loading data sets and calib parameters from YAML files via ROS
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/print_utils.h>
#include <rct_ros_tools/loader_utils.h>
// To find 2D  observations from images
#include <rct_image_tools/image_utils.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_multi_static_camera_only.h>
#include <rct_optimizations/extrinsic_multi_static_camera_wrist_only.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/experimental/multi_camera_pnp.h>
#include <pluginlib/class_loader.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

static void reproject(const Eigen::Isometry3d& base_to_target,
                      const std::vector<Eigen::Isometry3d>& base_to_camera,
                      const std::vector<CameraIntrinsics>& intr,
                      const cv::Mat& image,
                      const std::vector<Correspondence2D3D::Set>& correspondence_sets)
{

  Eigen::Isometry3d camera_to_target = base_to_camera[0].inverse() * base_to_target;

  std::vector<Eigen::Vector3d> target_points;
  for (const Correspondence2D3D::Set& corr_set : correspondence_sets)
  {
    for (const Correspondence2D3D& corr : corr_set)
    {
      target_points.push_back(corr.in_target);
    }
  }
  std::vector<cv::Point2d> reprojections = getReprojections(camera_to_target, intr[0], target_points);

  cv::Mat before_frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), before_frame);

  MultiCameraPnPProblem pb;
  pb.base_to_camera = base_to_camera;
  pb.base_to_target_guess = base_to_target;
  pb.image_observations = correspondence_sets;
  pb.intr = intr;

  MultiCameraPnPResult r = optimize(pb);
  printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  printNewLine();

  printTransform(camera_to_target, "Camera 0", "Target", "CAMERA 0 TO TARGET");
  printNewLine();

  Eigen::Isometry3d result_camera_to_target = base_to_camera[0].inverse() * r.base_to_target;
  printTransformDiff(camera_to_target, result_camera_to_target, "Camera 0", "Target", "PNP");
  printNewLine();

  reprojections = getReprojections(result_camera_to_target, intr[0], target_points);
  cv::Mat after_frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 255, 0), after_frame);

  cv::imshow("repr_before", before_frame);
  cv::imshow("repr_after", after_frame);
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
  ros::init(argc, argv, "multi_static_camera_multi_step_extrinsic");
  ros::NodeHandle pnh("~");

  try
  {
    std::size_t num_of_cameras = static_cast<std::size_t>(get<int>(pnh, "num_of_cameras"));

    ExtrinsicMultiStaticCameraMovingTargetWristOnlyProblem problem_wrist_def;
    ExtrinsicMultiStaticCameraOnlyProblem problem_def;
    std::vector<std::string> data_path;
    std::vector<ExtrinsicDataSet> maybe_data_set;

    data_path.resize(num_of_cameras);
    maybe_data_set.resize(num_of_cameras);
    problem_def.intr.resize(num_of_cameras);
    problem_def.base_to_camera_guess.resize(num_of_cameras);
    problem_def.image_observations.resize(num_of_cameras);
    for (std::size_t c = 0; c < num_of_cameras; ++c)
    {
      // Load the data set path from ROS param
      std::string param_base = "camera_" + std::to_string(c) + "/";
      std::string param_name = param_base + "data_path";
      data_path[c] = get<std::string>(pnh, param_name);

      // Attempt to load the data set from the specified path
      boost::optional<ExtrinsicDataSet> data_set = parseFromFile(data_path[c]);
      if (!data_set)
        throw std::runtime_error("Failed to parse data set from path = " + data_path[c]);
      maybe_data_set[c] = *data_set;

      // Load the camera intrinsics from the parameter server. Intr will get
      // reset if such a parameter was set
      param_name = param_base + "intrinsics";
      problem_def.intr[c].fx() = 1411.0;
      problem_def.intr[c].fy() = 1408.0;
      problem_def.intr[c].cx() = 807.2;
      problem_def.intr[c].cy() = 615.0;
      if (!loadIntrinsics(pnh, param_name, problem_def.intr[c]))
        throw std::runtime_error("Unable to load camera intrinsics from the '" + param_name + "' parameter struct");

      param_name = param_base + "base_to_camera_guess";
      // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
      if (!loadPose(pnh, param_name, problem_def.base_to_camera_guess[c]))
        throw std::runtime_error("Unable to load guess for base to camera from the '" + param_name +
                                 "' parameter struct");
    }

    bool fix_first_camera = get<bool>(pnh, "fix_first_camera");

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(toYAML(target_finder_config));

    Eigen::Isometry3d wrist_to_target;
    if (!loadPose(pnh, "wrist_to_target_guess", wrist_to_target))
      throw std::runtime_error("Unable to load guess for wrist to target from the 'wrist_to_target_guess' parameter struct");

    ExtrinsicCorrespondenceDataSet corr_data_set(maybe_data_set, *target_finder, true);

    // build problem
    problem_def.fix_first_camera = fix_first_camera;
    for (std::size_t i = 0; i < corr_data_set.getImageCount(); ++i)
    {
      // Currently requires that an image is found by all cameras but it maybe should
      // only require that an image is seen by at least two cameras.
      if (corr_data_set.getImageCameraCount(i) == corr_data_set.getCameraCount())
      {
        problem_def.base_to_target_guess.push_back(maybe_data_set[0].tool_poses[i] * wrist_to_target);
        problem_wrist_def.wrist_poses.push_back(maybe_data_set[0].tool_poses[i]);
        for (std::size_t c = 0; c < corr_data_set.getCameraCount(); ++c)
        {
          problem_def.image_observations[c].push_back(corr_data_set.getCorrespondenceSet(c, i));
        }
      }
    }

    // Run optimization
    printTitle("Running calibration for only cameras");

    ExtrinsicMultiStaticCameraOnlyResult
        opt_result = optimize(problem_def);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    for (std::size_t c = 0; c < num_of_cameras; ++c)
    {
      // Load the data set path from ROS param
      std::string param_base = "camera_" + std::to_string(c);

      Eigen::Isometry3d t = opt_result.base_to_camera[c];
      printTransform(t, "Base", "Camera (" + param_base + ")", "Base To Camera (" + param_base + ")");
      printNewLine();

      t = opt_result.base_to_camera[0].inverse()  * t;
      printTransform(t, "Camera 0", "Camera (" + param_base + ")", "Camera 0 to Camera " + std::to_string(c));
      printNewLine();
    }

    // Run optimization
    printTitle("Running calibration for wrist only using camera only results");

    problem_wrist_def.intr = problem_def.intr;
    problem_wrist_def.wrist_to_target_guess = wrist_to_target;
    problem_wrist_def.image_observations = problem_def.image_observations;
    problem_wrist_def.base_to_camera_guess = opt_result.base_to_camera;
    ExtrinsicMultiStaticCameraMovingTargetWristOnlyResult
        opt_wrist_only_result = optimize(problem_wrist_def);

    // Report results
    printOptResults(opt_wrist_only_result.converged, opt_wrist_only_result.initial_cost_per_obs, opt_wrist_only_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d t = opt_wrist_only_result.wrist_to_target;
    printTransform(t, "Wrist", "Target", "Wrist to Target");
    printNewLine();

    for (std::size_t c = 0; c < num_of_cameras; ++c)
    {
      // Load the data set path from ROS param
      std::string param_base = "camera_" + std::to_string(c);

      t = opt_wrist_only_result.base_to_camera[c];
      printTransform(t, "Base", "Camera (" + param_base + ")", "Base To Camera (" + param_base + ")");
      printNewLine();

      t = opt_wrist_only_result.base_to_camera[0].inverse() * t;
      printTransform(t, "Camera 0", "Camera (" + param_base + ")", "Camera 0 to Camera " + std::to_string(c));
      printNewLine();
    }

    printTitle("REPROJECTION ERROR");

    for (std::size_t i = 0; i < problem_wrist_def.wrist_poses.size(); ++i)
    {
      std::vector<Correspondence2D3D::Set> corr_set;
      std::vector<Eigen::Isometry3d> base_to_camera;
      Eigen::Isometry3d base_to_target;
      std::vector<CameraIntrinsics> intr;

      corr_set.reserve(num_of_cameras);
      base_to_camera.reserve(num_of_cameras);
      intr.reserve(num_of_cameras);

      base_to_target = problem_wrist_def.wrist_poses[i] * opt_wrist_only_result.wrist_to_target;

      for (std::size_t c = 0; c < num_of_cameras; ++c)
      {
        base_to_camera.push_back(opt_wrist_only_result.base_to_camera[c]);
        intr.push_back(problem_def.intr[c]);
        corr_set.push_back(problem_def.image_observations[c][i]);
      }

      printTitle("REPROJECT IMAGE " + std::to_string(i));
      reproject(base_to_target, base_to_camera, intr, maybe_data_set[0].images[i], corr_set);
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
