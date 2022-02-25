/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/experimental/multi_camera_pnp.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_common/print_utils.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/loader_utils.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;
using namespace rct_common;

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
  // Report results
  printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  printNewLine();

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  printTransform(base_to_target, "Base", "Target", "BASE TO TARGET");
  printNewLine();

  printTransform(r.base_to_target, "Base", "Target", "PNP");
  printNewLine();

  printTransformDiff(base_to_target, r.base_to_target, "Base", "Target", "PNP Diff");
  printNewLine();

  Eigen::Isometry3d result_camera_to_target = base_to_camera[0].inverse() * r.base_to_target;
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
  ros::init(argc, argv, "solve_multi_camera_pnp_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  try
  {
    std::size_t num_of_cameras = static_cast<std::size_t>(get<int>(pnh, "num_of_cameras"));

    std::vector<Eigen::Isometry3d> base_to_camera;
    std::vector<CameraIntrinsics> intr;
    std::vector<std::string> data_path;
    std::vector<ExtrinsicDataSet> maybe_data_set;

    data_path.resize(num_of_cameras);
    maybe_data_set.resize(num_of_cameras);
    intr.resize(num_of_cameras);
    base_to_camera.resize(num_of_cameras);
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
      intr[c].fx() = 1411.0;
      intr[c].fy() = 1408.0;
      intr[c].cx() = 807.2;
      intr[c].cy() = 615.0;
      if (!loadIntrinsics(pnh, param_name, intr[c]))
        throw std::runtime_error("Unable to load camera intrinsics from the '" + param_name + "' parameter struct");

      param_name = param_base + "base_to_camera";
      // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
      if (!loadPose(pnh, param_name, base_to_camera[c]))
        throw std::runtime_error("Unable to load guess for base to camera from the '" + param_name +
                                 "' parameter struct");
    }

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(toYAML(target_finder_config));

    ExtrinsicCorrespondenceDataSet corr_data_set(maybe_data_set, *target_finder, true);

    for (std::size_t i = 0; i < corr_data_set.getImageCount(); ++i)
    {
      if (corr_data_set.getImageCameraCount(i) == corr_data_set.getCameraCount())
      {
        std::vector<Correspondence2D3D::Set> corr_set;
        for (std::size_t c = 0; c < num_of_cameras; ++c)
        {
          corr_set.push_back(corr_data_set.getCorrespondenceSet(c, i));
        }

        printTitle("REPROJECT IMAGE " + std::to_string(i));
        reproject(maybe_data_set[0].tool_poses[i], base_to_camera, intr, maybe_data_set[0].images[i], corr_set);
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
