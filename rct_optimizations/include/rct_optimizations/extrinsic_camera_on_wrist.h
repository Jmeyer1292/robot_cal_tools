#ifndef EXTRINSIC_CAMERA_ON_WRIST_H
#define EXTRINSIC_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <vector>
#include <Eigen/Dense>

namespace rct_optimizations
{

struct ObservationPair
{
  Point3d in_target;
  Observation2d in_image;
};

using ObservationSet = std::vector<ObservationPair>;

struct ExtrinsicCameraOnWristProblem
{
  CameraIntrinsics intr;
  std::vector<Pose6d> wrist_poses;
  std::vector<ObservationSet> image_observations;

  Pose6d base_to_target_guess;
  Pose6d wrist_to_camera_guess;
};

struct ExtrinsicCameraOnWristResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Pose6d base_to_target;
  Pose6d wrist_to_camera;
};

ExtrinsicCameraOnWristResult optimize(const ExtrinsicCameraOnWristProblem& params);

}

#endif
