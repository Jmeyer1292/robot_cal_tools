#ifndef EXTRINSIC_CAMERA_ON_WRIST_H
#define EXTRINSIC_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>
#include <vector>

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
}

#endif
