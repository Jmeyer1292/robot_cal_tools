/*
 * This file is meant to provide an example of solving the PnP problem using OpenCV's functions and with Ceres
 * cost functions. This is a handy utility to have for doing stuff like quantifying the error of the system
 * after calibration
 */
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/pnp.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/print_utils.h>
#include <rct_ros_tools/loader_utils.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

std::string WINDOW = "window";

static Eigen::Isometry3d solveCVPnP(const CameraIntrinsics& intr,
                                    const Correspondence2D3D::Set& correspondences)
{
  cv::Mat cam_matrix (3, 3, cv::DataType<double>::type);
  cv::setIdentity(cam_matrix);

  cam_matrix.at<double>(0, 0) = intr.fx();
  cam_matrix.at<double>(1, 1) = intr.fy();
  cam_matrix.at<double>(0, 2) = intr.cx();
  cam_matrix.at<double>(1, 2) = intr.cy();

  printCameraIntrinsics(intr.values, "Camera Intrinsics");
  printNewLine();

  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> target_points;
  image_points.reserve(correspondences.size());
  target_points.reserve(correspondences.size());
  for (const Correspondence2D3D& corr : correspondences)
  {
    image_points.push_back(cv::Point2d(corr.in_image.x(), corr.in_image.y()));
    target_points.push_back(cv::Point3d(corr.in_target.x(), corr.in_target.y(), corr.in_target.z()));
  }

  cv::Mat rvec (3, 1, cv::DataType<double>::type);
  cv::Mat tvec (3, 1, cv::DataType<double>::type);
  cv::solvePnP(target_points, image_points, cam_matrix, cv::noArray(), rvec, tvec);

  Eigen::Vector3d rr (Eigen::Vector3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)));
  Eigen::Isometry3d result(Eigen::AngleAxisd(rr.norm(), rr.normalized()));
  result.translation() = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

  printTransform(result, "Camera", "Target", "OpenCV solvePNP");
  printNewLine();

  return result;
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
  ros::init(argc, argv, "solve_pnp_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  try
  {
    // Load the image
    std::string image_path = get<std::string>(pnh, "image_path");
    cv::Mat mat = cv::imread(image_path);

    // Load the camera intrinsics from the parameter server
    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(toYAML(target_finder_config));

    rct_image_tools::TargetFeatures target_features = target_finder->findTargetFeatures(mat);
    if (target_features.empty())
      throw std::runtime_error("Failed to find any target features");
    ROS_INFO_STREAM("Found " << target_features.size() << " target features");

    cv::Mat show = target_finder->drawTargetFeatures(mat, target_features);
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
    cv::imshow(WINDOW, show);
    cv::waitKey();

    // Solve with OpenCV
    solveCVPnP(intr, target_finder->target().createCorrespondences(target_features));

    // Solve with some native RCT function (for learning)
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess = guess * Eigen::Translation3d(0, 0, 0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    PnPProblem params;
    params.intr = intr;
    params.camera_to_target_guess = guess;
    params.correspondences = target_finder->target().createCorrespondences(target_features);

    PnPResult pnp_result = optimize(params);

    printOptResults(pnp_result.converged, pnp_result.initial_cost_per_obs, pnp_result.final_cost_per_obs);
    printNewLine();

    printTransform(pnp_result.camera_to_target, "Camera", "Target", "RCT CAMERA TO TARGET");
    printNewLine();
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
