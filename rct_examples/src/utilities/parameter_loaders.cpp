#include "rct_examples/parameter_loaders.h"

template<typename T>
static bool read(XmlRpc::XmlRpcValue& xml, const std::string& key, T& value)
{
  if (!xml.hasMember(key)) return false;
  value = static_cast<T>(xml[key]);
  return true;
}

bool rct_examples::loadTarget(const ros::NodeHandle& nh, const std::string& key,
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

bool rct_examples::loadIntrinsics(const ros::NodeHandle& nh, const std::string& key,
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

bool rct_examples::loadPose(const ros::NodeHandle& nh, const std::string& key,
                            Eigen::Affine3d& pose)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) return false;

  pose = Eigen::Affine3d::Identity();
  double x, y, z, qx, qy, qz, qw;
  if (!read(xml, "x", x)) return false;
  if (!read(xml, "y", y)) return false;
  if (!read(xml, "x", z)) return false;
  if (!read(xml, "qx", qx)) return false;
  if (!read(xml, "qy", qy)) return false;
  if (!read(xml, "qz", qz)) return false;
  if (!read(xml, "qw", qw)) return false;

  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

  return true;
}
