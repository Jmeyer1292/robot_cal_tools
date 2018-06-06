#ifndef RCT_EXTRINSIC_STATIC_CAMERA_H
#define RCT_EXTRINSIC_STATIC_CAMERA_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{

struct ExtrinsicStaticCameraMovingTargetProblem
{
  CameraIntrinsics intr;
  std::vector<Eigen::Affine3d> wrist_poses;
  std::vector<ObservationSet> image_observations;

  Eigen::Affine3d wrist_to_target_guess;
  Eigen::Affine3d base_to_camera_guess;
};

struct ExtrinsicStaticCameraMovingTargetResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Affine3d base_to_target;
  Eigen::Affine3d wrist_to_camera;
};

ExtrinsicStaticCameraMovingTargetProblem optimize(const ExtrinsicStaticCameraMovingTargetResult& params);

}

#endif // RCT_EXTRINSIC_STATIC_CAMERA_H
