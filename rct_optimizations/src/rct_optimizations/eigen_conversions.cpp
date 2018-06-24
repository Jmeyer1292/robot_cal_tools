#include "rct_optimizations/eigen_conversions.h"

rct_optimizations::Pose6d rct_optimizations::poseEigenToCal(const Eigen::Affine3d& pose)
{
  Pose6d p;
  p.x() = pose.translation().x();
  p.y() = pose.translation().y();
  p.z() = pose.translation().z();

  Eigen::AngleAxisd aa(pose.linear());
  Eigen::Vector3d a = aa.axis() * aa.angle();

  p.rx() = a.x();
  p.ry() = a.y();
  p.rz() = a.z();
  return p;
}

Eigen::Affine3d rct_optimizations::poseCalToEigen(const rct_optimizations::Pose6d& pose)
{
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.translation().x() = pose.x();
  p.translation().y() = pose.y();
  p.translation().z() = pose.z();

  const Eigen::Vector3d rr(pose.rx(), pose.ry(), pose.rz());
  const double dot_product = rr.dot(rr);

  // Check for 0-rotation
  if (dot_product > std::numeric_limits<double>::epsilon())
  {
    Eigen::AngleAxisd aa(rr.norm(), rr.normalized());
    p.linear() = aa.toRotationMatrix();
  }
  else
  {
    p.linear()(0, 0) = 1.0; // Logic taken from Ceres' own aaxis to rot matrix conversion
    p.linear()(1, 0) = rr[2];
    p.linear()(2, 0) = -rr[1];
    p.linear()(0, 1) = -rr[2];
    p.linear()(1, 1) = 1.0;
    p.linear()(2, 1) =  rr[0];
    p.linear()(0, 2) =  rr[1];
    p.linear()(1, 2) = -rr[0];
    p.linear()(2, 2) = 1.0;
  }

  return p;
}
