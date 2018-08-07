#include "rct_ros_tools/parameter_loaders.h"
#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>

template<typename T>
static bool read(XmlRpc::XmlRpcValue& xml, const std::string& key, T& value)
{
  if (!xml.hasMember(key)) return false;
  try {
    value = static_cast<T>(xml[key]);
  } catch (const XmlRpc::XmlRpcException& ex) {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }
  return true;
}

rct_image_tools::ModifiedCircleGridTarget rct_ros_tools::loadTarget(const ros::NodeHandle& nh, const std::string& key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) throw ros::InvalidParameterException(key);

  int rows = 0;
  int cols = 0;
  double spacing = 0.0;

  if (!read(xml, "rows", rows)) throw ros::InvalidParameterException(key + "/rows");
  if (!read(xml, "cols", cols)) throw ros::InvalidParameterException(key + "/cols");
  if (!read(xml, "spacing", spacing)) throw ros::InvalidParameterException(key + "/spacing");

  return rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
}

bool rct_ros_tools::loadTarget(const ros::NodeHandle& nh, const std::string& key,
                               rct_image_tools::ModifiedCircleGridTarget& target)
{
  try
  {
    target = loadTarget(nh, key);
  }
  catch (ros::InvalidParameterException &ex)
  {
    ROS_ERROR_STREAM("Failed to load target parameter: " << ex.what());
    return false;
  }
  return true;
}

rct_image_tools::ModifiedCircleGridTarget rct_ros_tools::loadTarget(const std::string &path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    int rows = n["target_definition"]["rows"].as<int>();
    int cols = n["target_definition"]["cols"].as<int>();
    double spacing = n["target_definition"]["spacing"].as<double>(); // (meters between dot centers)
    return rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool rct_ros_tools::loadTarget(const std::string& path, rct_image_tools::ModifiedCircleGridTarget& target)
{
  try
  {
    target = loadTarget(path);
  }
  catch (ros::InvalidParameterException &ex)
  {
    ROS_ERROR_STREAM("Failed to load target from file: " << ex.what());
    return false;
  }
  return true;
}

rct_optimizations::CameraIntrinsics rct_ros_tools::loadIntrinsics(const ros::NodeHandle &nh, const std::string &key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) throw ros::InvalidParameterException(key);

  rct_optimizations::CameraIntrinsics intr;
  if (!read(xml, "fx", intr.fx())) throw ros::InvalidParameterException(key + "/fx");
  if (!read(xml, "fy", intr.fy())) throw ros::InvalidParameterException(key + "/fy");
  if (!read(xml, "cx", intr.cx())) throw ros::InvalidParameterException(key + "/cx");
  if (!read(xml, "cy", intr.cy())) throw ros::InvalidParameterException(key + "/cy");

  return intr;
}

bool rct_ros_tools::loadIntrinsics(const ros::NodeHandle& nh, const std::string& key,
                                  rct_optimizations::CameraIntrinsics& intr)
{
  try
  {
    intr = loadIntrinsics(nh, key);
  }
  catch (ros::InvalidParameterException &ex)
  {
    ROS_ERROR_STREAM("Failed to load intrisic parameter: " << ex.what());
    return false;
  }
  return true;
}

rct_optimizations::CameraIntrinsics rct_ros_tools::loadIntrinsics(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);

    rct_optimizations::CameraIntrinsics intrinsics;
    intrinsics.cx() = n["cx"].as<double>();
    intrinsics.cy() = n["cy"].as<double>();
    intrinsics.fx() = n["fx"].as<double>();
    intrinsics.fy() = n["fy"].as<double>();
    return intrinsics;
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool rct_ros_tools::loadIntrinsics(const std::string& path, rct_optimizations::CameraIntrinsics& intrinsics)
{
  try
  {
    intrinsics = loadIntrinsics(path);
  }
  catch (BadFileException &ex)
  {
    ROS_ERROR_STREAM("Failed to load intrinsics from file: " << ex.what());
    return false;
  }
  return true;
}

Eigen::Isometry3d rct_ros_tools::loadPose(const ros::NodeHandle &nh, const std::string &key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) throw ros::InvalidParameterException(key);

  double x, y, z, qx, qy, qz, qw;
  if (!read(xml, "x", x)) throw ros::InvalidParameterException(key + "/x");
  if (!read(xml, "y", y)) throw ros::InvalidParameterException(key + "/y");
  if (!read(xml, "z", z)) throw ros::InvalidParameterException(key + "/z");
  if (!read(xml, "qx", qx)) throw ros::InvalidParameterException(key + "/qx");
  if (!read(xml, "qy", qy)) throw ros::InvalidParameterException(key + "/qy");
  if (!read(xml, "qz", qz)) throw ros::InvalidParameterException(key + "/qz");
  if (!read(xml, "qw", qw)) throw ros::InvalidParameterException(key + "/qw");

  Eigen::Isometry3d pose =
      Eigen::Translation3d(Eigen::Vector3d(x, y, z)) *
      Eigen::AngleAxisd(Eigen::Quaterniond(qw, qx, qy, qz));
  return pose;
}

bool rct_ros_tools::loadPose(const ros::NodeHandle& nh, const std::string& key, Eigen::Isometry3d& pose)
{
  try
  {
    pose = loadPose(nh, key);
  }
  catch (ros::InvalidParameterException &ex)
  {
    ROS_ERROR_STREAM("Failed to load pose parameter: " << ex.what());
    return false;
  }
  return true;
}

Eigen::Isometry3d rct_ros_tools::loadPose(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);

    Eigen::Vector3d position;
    position(0) = n["x"].as<double>();
    position(1) = n["y"].as<double>();
    position(2) = n["z"].as<double>();

    Eigen::Quaterniond quat;
    quat.w() = n["qw"].as<double>();
    quat.x() = n["qx"].as<double>();
    quat.y() = n["qy"].as<double>();
    quat.z() = n["qz"].as<double>();

    Eigen::Isometry3d pose = Eigen::Translation3d(position) * Eigen::AngleAxisd(quat);
    return pose;
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool rct_ros_tools::loadPose(const std::string& path, Eigen::Isometry3d& pose)
{
  try
  {
    pose = loadPose(path);
  }
  catch (BadFileException &ex)
  {
    ROS_ERROR_STREAM("Failed to load pose from file: " << ex.what());
    return false;
  }
  return true;
}
