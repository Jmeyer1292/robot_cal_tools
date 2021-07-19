// Utilities for loading data sets and calib parameters from YAML files via ROS
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/print_utils.h>
// To find 2D  observations from images
#include <rct_image_tools/image_utils.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_multi_static_camera.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pluginlib/class_loader.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/experimental/multi_camera_pnp.h>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

static void reproject(const Eigen::Isometry3d& base_to_target, const std::vector<Eigen::Isometry3d>& base_to_camera,
                      const std::vector<CameraIntrinsics>& intr, const cv::Mat& image,
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
  // Report results
  printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  printNewLine();

  printTransform(camera_to_target, "Camera 0", "Target", "CAMERA 0 TO TARGET");
  printNewLine();

  Eigen::Isometry3d result_camera_to_target = base_to_camera[0].inverse() * r.base_to_target;
  printTransform(result_camera_to_target, "Camera 0", "Target", "PNP");
  printNewLine();

  printTransformDiff(camera_to_target, result_camera_to_target, "Camera 0", "Target", "PNP DIFF");
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
  ros::init(argc, argv, "multi_static_camera_extrinsic");
  ros::NodeHandle pnh("~");

  try
  {
    std::size_t num_of_cameras = static_cast<std::size_t>(get<int>(pnh, "num_of_cameras"));

    ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;
    std::vector<std::string> data_path;
    std::vector<ExtrinsicDataSet> maybe_data_set;

    data_path.resize(num_of_cameras);
    maybe_data_set.resize(num_of_cameras);
    problem_def.intr.resize(num_of_cameras);
    problem_def.base_to_camera_guess.resize(num_of_cameras);
    problem_def.image_observations.resize(num_of_cameras);
    problem_def.wrist_poses.resize(num_of_cameras);
    for (std::size_t c = 0; c < num_of_cameras; ++c)
    {
      // Load the data set path from ROS param
      std::string param_base = "camera_" + std::to_string(c) + "/";
      std::string param_name = param_base + "data_path";
      data_path[c] = get<std::string>(pnh, param_name);

      // Attempt to load the data set from the specified path
      boost::optional<ExtrinsicDataSet> data_set = *parseFromFile(data_path[c]);
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
        throw std::runtime_error("Unable to load guess for base to camera from the '" + param_name + "' parameter struct");
    }

    if (!loadPose(pnh, "wrist_to_target_guess", problem_def.wrist_to_target_guess))
      throw std::runtime_error("Unable to load guess for wrist to target from the 'wrist_to_target_guess' parameter struct");

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(target_finder_config);

    // Load the data set
    ExtrinsicCorrespondenceDataSet corr_data_set(maybe_data_set, *target_finder, true);

    // build problem
    for (std::size_t c = 0; c < corr_data_set.getCameraCount(); ++c)
    {
      for (std::size_t i = 0; i < corr_data_set.getImageCount(); ++i)
      {
        if (corr_data_set.foundCorrespondence(c, i))
        {
          const ExtrinsicDataSet& data_set = maybe_data_set[c];
          problem_def.wrist_poses[c].push_back(data_set.tool_poses[i]);
          problem_def.image_observations[c].push_back(corr_data_set.getCorrespondenceSet(c, i));
        }
      }
    }

    // Run optimization
    ExtrinsicMultiStaticCameraMovingTargetResult opt_result = optimize(problem_def);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d t = opt_result.wrist_to_target;
    printTransform(t, "Wrist", "Target", "WRIST TO TARGET");
    printNewLine();

    for (std::size_t c = 0; c < num_of_cameras; ++c)
    {
      // Load the data set path from ROS param
      std::string param_base = "camera_" + std::to_string(c);
      t = opt_result.base_to_camera[c];
      printTransform(t, "Base", " Camera (" + param_base + ")", "BASE TO CAMERA (" + param_base + ")");
      printNewLine();

      t = opt_result.base_to_camera[0].inverse() * t;
      printTransform(t, "Camera 0", " Camera " + std::to_string(c), "CAMERA 0 TO CAMERA (" + param_base + ")");
      printNewLine();
    }

    printTitle("REPROJECTION ERROR");

    for (std::size_t i = 0; i < maybe_data_set[0].images.size(); ++i)
    {
      std::vector<Correspondence2D3D::Set> corr_set;
      std::vector<Eigen::Isometry3d> base_to_camera;
      Eigen::Isometry3d base_to_wrist;
      std::vector<CameraIntrinsics> intr;
      cv::Mat image;

      corr_set.reserve(num_of_cameras);
      base_to_camera.reserve(num_of_cameras);
      intr.reserve(num_of_cameras);

      base_to_wrist = maybe_data_set[0].tool_poses[i];

      std::size_t cnt = 0;
      for (std::size_t c = 0; c < num_of_cameras; ++c)
      {
        if (corr_data_set.foundCorrespondence(c, i))
        {
          base_to_camera.push_back(opt_result.base_to_camera[c]);
          intr.push_back(problem_def.intr[c]);
          corr_set.push_back(corr_data_set.getCorrespondenceSet(c, i));
          if (cnt == 0)
          {
            image = maybe_data_set[c].images[i];
          }

          ++cnt;
        }
        else
        {
          continue;
        }
      }

      if (cnt >= 2)
      {
        printTitle("REPROJECT IMAGE " + std::to_string(i));
        reproject(base_to_wrist * opt_result.wrist_to_target, base_to_camera, intr, image, corr_set);
      }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
