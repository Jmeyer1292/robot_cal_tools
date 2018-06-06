#ifndef EXTRINSIC_3D_CAMERA_ON_WRIST_H
#define EXTRINSIC_3D_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{

struct Extrinsic3DCameraOnWristProblem
{
  std::vector<Eigen::Affine3d> wrist_poses;
  std::vector<Observation3DSet> observations;

  Eigen::Affine3d base_to_target_guess;
  Eigen::Affine3d wrist_to_camera_guess;
};

struct Extrinsic3DCameraOnWristResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Affine3d base_to_target;
  Eigen::Affine3d wrist_to_camera;
};

Extrinsic3DCameraOnWristResult optimize(const Extrinsic3DCameraOnWristProblem& params);

}

#endif // EXTRINSIC_3D_CAMERA_ON_WRIST_H
