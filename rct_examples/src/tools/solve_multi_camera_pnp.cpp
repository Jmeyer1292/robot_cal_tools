/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/experimental/multi_camera_pnp.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/print_utils.h>

static void reproject(const Eigen::Affine3d& base_to_target,
                      const std::vector<Eigen::Affine3d>& base_to_camera,
                      const std::vector<rct_optimizations::CameraIntrinsics>& intr,
                      const rct_image_tools::ModifiedCircleGridTarget& target,
                      const cv::Mat& image,
                      const std::vector<rct_optimizations::CorrespondenceSet>& corr)
{

  Eigen::Affine3d camera_to_target = base_to_camera[0].inverse() * base_to_target;
  std::vector<cv::Point2d> reprojections = rct_image_tools::getReprojections(camera_to_target, intr[0], target.points);

  cv::Mat before_frame = image.clone();
  rct_image_tools::drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), before_frame);

  rct_optimizations::MultiCameraPnPProblem pb;
  pb.base_to_camera = base_to_camera;
  pb.base_to_target_guess = base_to_target;
  pb.image_observations = corr;
  pb.intr = intr;

  rct_optimizations::MultiCameraPnPResult r = rct_optimizations::optimize(pb);
  // Report results
  rct_ros_tools::printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  rct_ros_tools::printTransform(base_to_target, "Base", "Target", "BASE TO TARGET");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTransform(r.base_to_target, "Base", "Target", "PNP");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printTransformDiff(base_to_target, r.base_to_target, "Base", "Target", "PNP Diff");
  rct_ros_tools::printNewLine();

  Eigen::Affine3d result_camera_to_target = base_to_camera[0].inverse() * r.base_to_target;
  reprojections = rct_image_tools::getReprojections(result_camera_to_target, intr[0], target.points);

  cv::Mat after_frame = image.clone();
  rct_image_tools::drawReprojections(reprojections, 3, cv::Scalar(0, 255, 0), after_frame);

  cv::imshow("repr_before", before_frame);
  cv::imshow("repr_after", after_frame);
  cv::waitKey();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "solve_multi_camera_pnp_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  int camera_count;
  if (!pnh.getParam("num_of_cameras", camera_count))
  {
    ROS_ERROR("Must set '%s' parameter", "num_of_cameras");
    return 1;
  }
  std::size_t num_of_cameras = camera_count;

  std::vector<Eigen::Affine3d> base_to_camera;
  std::vector<rct_optimizations::CameraIntrinsics> intr;
  std::vector<std::string> data_path;
  std::string target_path;
  std::vector<rct_ros_tools::ExtrinsicDataSet> maybe_data_set;

  data_path.resize(num_of_cameras);
  maybe_data_set.resize(num_of_cameras);
  intr.resize(num_of_cameras);
  base_to_camera.resize(num_of_cameras);
  for (std::size_t c = 0; c < num_of_cameras; ++c)
  {
    // Load the data set path from ROS param
    std::string param_base = "camera_" + std::to_string(c) + "/";
    std::string param_name = param_base + "data_path";
    if (!pnh.getParam(param_name, data_path[c]))
    {
      ROS_ERROR("Must set '%s' parameter", param_name.c_str());
      return 1;
    }

    // Attempt to load the data set from the specified path
    boost::optional<rct_ros_tools::ExtrinsicDataSet> data_set = rct_ros_tools::parseFromFile(data_path[c]);
    if (!data_set)
    {
      ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path[c]);
      return 2;
    }
    maybe_data_set[c] = *data_set;

    // Load the camera intrinsics from the parameter server. Intr will get
    // reset if such a parameter was set
    param_name = param_base + "intrinsics";
    intr[c].fx() = 1411.0;
    intr[c].fy() = 1408.0;
    intr[c].cx() = 807.2;
    intr[c].cy() = 615.0;
    if (!rct_ros_tools::loadIntrinsics(pnh, param_name, intr[c]))
    {
      ROS_WARN("Unable to load camera intrinsics from the '%s' parameter struct", param_name.c_str());
    }

    param_name = param_base + "base_to_camera";
    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    if (!rct_ros_tools::loadPose(pnh, param_name, base_to_camera[c]))
    {
      ROS_WARN("Unable to load guess for base to camera from the '%s' parameter struct", param_name.c_str());
    }
  }

  if (!pnh.getParam("target_path", target_path))
  {
    ROS_ERROR("Must set '%s' parameter", "target_path");
    return 1;
  }

  // Load target definition from parameter server. Target will get
  // reset if such a parameter was set.
  rct_image_tools::ModifiedCircleGridTarget target(5, 5, 0.015);
  if (!rct_ros_tools::loadTarget(target_path, target))
  {
    ROS_WARN_STREAM("Unable to load target file from the 'target_path' parameter");
  }

  // Lets create a class that will search for the target in our raw images.
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  rct_ros_tools::ExtrinsicCorrespondenceDataSet corr_data_set(maybe_data_set, obs_finder, true);

  for (std::size_t i = 0; i < corr_data_set.getImageCount(); ++i)
  {
    if (corr_data_set.getImageCameraCount(i) == corr_data_set.getCameraCount())
    {
      std::vector<rct_optimizations::CorrespondenceSet> corr_set;
      for (std::size_t c = 0; c < num_of_cameras; ++c)
      {
        corr_set.push_back(corr_data_set.getCorrespondenceSet(c, i));
      }

      rct_ros_tools::printTitle("REPROJECT IMAGE " + std::to_string(i));
      reproject(maybe_data_set[0].tool_poses[i], base_to_camera,
                intr, target, maybe_data_set[0].images[i], corr_set);
    }
  }
}

