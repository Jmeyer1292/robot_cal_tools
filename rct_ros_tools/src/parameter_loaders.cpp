#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/exceptions.h>
#include <rct_optimizations/serialization/eigen.h>
#include <rct_optimizations/serialization/types.h>

#include <xmlrpcpp/XmlRpcException.h>

template <typename T>
static bool read(XmlRpc::XmlRpcValue& xml, const std::string& key, T& value)
{
  if (!xml.hasMember(key))
    return false;
  try
  {
    value = static_cast<T>(xml[key]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }
  return true;
}

namespace rct_ros_tools
{
rct_optimizations::CameraIntrinsics loadIntrinsics(const ros::NodeHandle& nh, const std::string& key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml))
    throw ros::InvalidParameterException(key);

  rct_optimizations::CameraIntrinsics intr;
  if (!read(xml, "fx", intr.fx()))
    throw ros::InvalidParameterException(key + "/fx");
  if (!read(xml, "fy", intr.fy()))
    throw ros::InvalidParameterException(key + "/fy");
  if (!read(xml, "cx", intr.cx()))
    throw ros::InvalidParameterException(key + "/cx");
  if (!read(xml, "cy", intr.cy()))
    throw ros::InvalidParameterException(key + "/cy");

  return intr;
}

bool loadIntrinsics(const ros::NodeHandle& nh, const std::string& key, rct_optimizations::CameraIntrinsics& intr)
{
  try
  {
    intr = loadIntrinsics(nh, key);
  }
  catch (ros::InvalidParameterException& ex)
  {
    ROS_ERROR_STREAM("Failed to load intrisic parameter: " << ex.what());
    return false;
  }
  return true;
}

rct_optimizations::CameraIntrinsics loadIntrinsics(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    return n.as<rct_optimizations::CameraIntrinsics>();
  }
  catch (YAML::Exception& ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool loadIntrinsics(const std::string& path, rct_optimizations::CameraIntrinsics& intrinsics)
{
  try
  {
    intrinsics = loadIntrinsics(path);
  }
  catch (BadFileException& ex)
  {
    ROS_ERROR_STREAM("Failed to load intrinsics from file: " << ex.what());
    return false;
  }
  return true;
}

Eigen::Isometry3d loadPose(const ros::NodeHandle& nh, const std::string& key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml))
    throw ros::InvalidParameterException(key);

  double x, y, z, qx, qy, qz, qw;
  if (!read(xml, "x", x))
    throw ros::InvalidParameterException(key + "/x");
  if (!read(xml, "y", y))
    throw ros::InvalidParameterException(key + "/y");
  if (!read(xml, "z", z))
    throw ros::InvalidParameterException(key + "/z");
  if (!read(xml, "qx", qx))
    throw ros::InvalidParameterException(key + "/qx");
  if (!read(xml, "qy", qy))
    throw ros::InvalidParameterException(key + "/qy");
  if (!read(xml, "qz", qz))
    throw ros::InvalidParameterException(key + "/qz");
  if (!read(xml, "qw", qw))
    throw ros::InvalidParameterException(key + "/qw");

  Eigen::Isometry3d pose =
      Eigen::Translation3d(Eigen::Vector3d(x, y, z)) * Eigen::AngleAxisd(Eigen::Quaterniond(qw, qx, qy, qz));
  return pose;
}

bool loadPose(const ros::NodeHandle& nh, const std::string& key, Eigen::Isometry3d& pose)
{
  try
  {
    pose = loadPose(nh, key);
  }
  catch (ros::InvalidParameterException& ex)
  {
    ROS_ERROR_STREAM("Failed to load pose parameter: " << ex.what());
    return false;
  }
  return true;
}

Eigen::Isometry3d loadPose(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    return n.as<Eigen::Isometry3d>();
  }
  catch (YAML::Exception& ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool loadPose(const std::string& path, Eigen::Isometry3d& pose)
{
  try
  {
    pose = loadPose(path);
  }
  catch (BadFileException& ex)
  {
    ROS_ERROR_STREAM("Failed to load pose from file: " << ex.what());
    return false;
  }
  return true;
}

}  // namespace rct_ros_tools
