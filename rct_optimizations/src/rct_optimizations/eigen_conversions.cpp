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

  Eigen::Vector3d rr(pose.rx(), pose.ry(), pose.rz());
  double angle = rr.norm();

  Eigen::AngleAxisd aa(angle, rr.normalized());
  p.linear() = aa.toRotationMatrix();

  return p;
}
