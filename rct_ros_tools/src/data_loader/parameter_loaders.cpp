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

bool rct_ros_tools::loadTarget(const ros::NodeHandle& nh, const std::string& key,
                               rct_image_tools::ModifiedCircleGridTarget& target)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) return false;

  int rows = 0;
  int cols = 0;
  double spacing = 0.0;

  if (!read(xml, "rows", rows)) return false;
  if (!read(xml, "cols", cols)) return false;
  if (!read(xml, "spacing", spacing)) return false;

  target = rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
  return true;
}

bool rct_ros_tools::loadTarget(const std::string& path, rct_image_tools::ModifiedCircleGridTarget& target)
{
  YAML::Node n = YAML::LoadFile(path);
  int rows = n["target_definition"]["rows"].as<int>();
  int cols = n["target_definition"]["cols"].as<int>();
  double spacing = n["target_definition"]["spacing"].as<double>(); // (meters between dot centers)

  target = rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
  return true;
}

bool rct_ros_tools::loadIntrinsics(const ros::NodeHandle& nh, const std::string& key,
                                  rct_optimizations::CameraIntrinsics& intr)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) return false;

  rct_optimizations::CameraIntrinsics temp_intr;
  if (!read(xml, "fx", temp_intr.fx())) return false;
  if (!read(xml, "fy", temp_intr.fy())) return false;
  if (!read(xml, "cx", temp_intr.cx())) return false;
  if (!read(xml, "cy", temp_intr.cy())) return false;

  intr = temp_intr;
  return true;
}

bool rct_ros_tools::loadPose(const ros::NodeHandle& nh, const std::string& key,
                            Eigen::Affine3d& pose)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) return false;

  pose = Eigen::Affine3d::Identity();
  double x, y, z, qx, qy, qz, qw;
  if (!read(xml, "x", x)) return false;
  if (!read(xml, "y", y)) return false;
  if (!read(xml, "z", z)) return false;
  if (!read(xml, "qx", qx)) return false;
  if (!read(xml, "qy", qy)) return false;
  if (!read(xml, "qz", qz)) return false;
  if (!read(xml, "qw", qw)) return false;

  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

  return true;
}

bool rct_ros_tools::loadPose(const std::string& path, Eigen::Affine3d& pose)
{
  YAML::Node n = YAML::LoadFile(path);
  Eigen::Vector3d position;

  position(0) = n["x"].as<double>();
  position(1) = n["y"].as<double>();
  position(2) = n["z"].as<double>();

  double qw, qx, qy, qz;
  qw = n["qw"].as<double>();
  qx = n["qx"].as<double>();
  qy = n["qy"].as<double>();
  qz = n["qz"].as<double>();

  pose = Eigen::Affine3d::Identity();
  pose.translation() = position;
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
  return true;
}
