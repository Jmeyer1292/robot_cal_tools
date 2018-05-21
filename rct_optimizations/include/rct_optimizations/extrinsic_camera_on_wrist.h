#ifndef EXTRINSIC_CAMERA_ON_WRIST_H
#define EXTRINSIC_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <vector>
#include <Eigen/Dense>

namespace rct_optimizations
{

struct ObservationPair
{
  Eigen::Vector3d in_target;
  Eigen::Vector2d in_image;
};

using ObservationSet = std::vector<ObservationPair>;

struct ExtrinsicCameraOnWristProblem
{
  CameraIntrinsics intr;
  std::vector<Eigen::Affine3d> wrist_poses;
  std::vector<ObservationSet> image_observations;

  Eigen::Affine3d base_to_target_guess;
  Eigen::Affine3d wrist_to_camera_guess;
};

struct ExtrinsicCameraOnWristResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Affine3d base_to_target;
  Eigen::Affine3d wrist_to_camera;
};

ExtrinsicCameraOnWristResult optimize(const ExtrinsicCameraOnWristProblem& params);

inline Pose6d poseEigenToCal(const Eigen::Affine3d& pose)
{
  Pose6d p;
  p.x() = pose.translation().x();
  p.y() = pose.translation().y();
  p.z() = pose.translation().z();

  Eigen::AngleAxisd aa (pose.linear());
  Eigen::Vector3d a = aa.axis() * aa.angle();

  p.rx() = a.x();
  p.ry() = a.y();
  p.rz() = a.z();
  return p;
}

inline Eigen::Affine3d poseCalToEigen(const Pose6d& pose)
{
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.translation().x() = pose.x();
  p.translation().y() = pose.y();
  p.translation().z() = pose.z();

  Eigen::Vector3d rr (pose.rx(), pose.ry(), pose.rz());
  double angle = rr.norm();

  Eigen::AngleAxisd aa(angle, rr.normalized());
  p.linear() = aa.toRotationMatrix();

  return p;
}
}

#endif
