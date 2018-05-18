#ifndef EXTRINSIC_CAMERA_ON_WRIST_H
#define EXTRINSIC_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <vector>

namespace rct_optimizations
{

struct ExtrinsicCameraOnWristParameters
{
  CameraIntrinsics intr;
  std::vector<Pose6d> wrist_poses;
  std::vector<std::vector<std::pair<Point3d, Observation2d>>> obs;

  Pose6d base_to_target_guess;
  Pose6d wrist_to_camera_guess;
};

struct ExtrinsicCameraOnWristResult
{
  bool converged;
  double cost_per_obs;

  Pose6d base_to_target;
  Pose6d wrist_to_camera;
};

ExtrinsicCameraOnWristResult optimize(const ExtrinsicCameraOnWristParameters& params);

}

#endif
